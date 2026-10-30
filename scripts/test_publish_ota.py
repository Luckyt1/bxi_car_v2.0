"""Exercise release ordering and authentication against a local HTTP server."""

import contextlib
import hashlib
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, unquote, urlsplit

from publish_ota import OpenList, PublishError, publish


class Site:
    token = 'unit-test-token-not-a-real-credential'
    password = 'unit-test-password-not-a-real-credential'
    public_directory = '/ELF3/bxi_arm_car2'

    def __init__(self):
        self.base_path = self.public_directory
        self.files = {}
        self.events = []
        self.fail_upload = False
        self.upload_http_code = 200
        self.omit_task_id = False
        self.upload_task_states = [2]
        self.tasks = {}
        self.task_polls = []
        self.task_info_code = 200
        self.task_id_prefix = 'upload-'
        self.private_downloads = False
        self.public_listing = True
        self.hidden_public_files = set()
        self.public_list_payloads = []
        self.corrupt_archive_download = False
        self.redirect_api = ''
        self.redirect_download = ''
        self.list_code = 200
        self.raw_url = ''
        site = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def respond(self, payload, code=200):
                body = json.dumps(payload).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', '0')))
                payload = json.loads(body) if body else {}
                site.events.append(('POST', self.path, payload.get('path'),
                                    bool(self.headers.get('Authorization'))))
                if site.redirect_api:
                    self.send_response(307)
                    self.send_header('Location', site.redirect_api)
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
                if self.path == '/api/auth/login':
                    if payload.get('password') != site.password:
                        # Deliberately echo secrets: the client must not log the
                        # server's untrusted error message.
                        self.respond({'code': 401, 'message': site.password + site.token})
                    else:
                        self.respond({'code': 200, 'data': {'token': site.token}})
                    return
                if self.path == '/api/fs/get':
                    if site.private_downloads:
                        self.respond({'code': 403, 'message': 'access denied'})
                        return
                    name = payload['path'].removeprefix(site.public_directory + '/')
                    if name not in site.files:
                        self.respond({'code': 500, 'message': 'object not found'})
                        return
                    self.respond({'code': 200, 'data': {
                        'name': name, 'size': len(site.files[name]), 'is_dir': False,
                        'raw_url': site.raw_url or site.url + '/download/' + name,
                    }})
                    return
                authorization = self.headers.get('Authorization')
                if self.path == '/api/fs/list' and not authorization:
                    site.public_list_payloads.append(payload)
                    if not site.public_listing:
                        self.respond({'code': 401, 'message': 'guest cannot list this directory'})
                        return
                    if payload != {'path': site.public_directory}:
                        self.respond({'code': 403, 'message': 'guest request differs from robot'})
                        return
                    public = {name: data for name, data in site.files.items()
                              if name not in site.hidden_public_files}
                    self.respond({'code': 200, 'data': {
                        'content': [{'name': name, 'size': len(data), 'is_dir': False}
                                    for name, data in public.items()],
                        'total': len(public),
                    }})
                    return
                if authorization != site.token:
                    self.respond({'code': 401})
                    return
                if self.path.startswith('/api/task/upload/info?'):
                    task_id = parse_qs(urlsplit(self.path).query).get('tid', [''])[0]
                    task = site.tasks.get(task_id)
                    if task is None:
                        self.respond({'code': 404})
                        return
                    states = task['states']
                    state = states.pop(0) if len(states) > 1 else states[0]
                    site.task_polls.append((task_id, state))
                    if state == 2 and site.task_info_code == 200:
                        site.files[task['name']] = task['body']
                    self.respond({'code': site.task_info_code,
                                  'message': site.token + site.password,
                                  'data': {'id': task_id, 'state': state,
                                           'progress': 100, 'status': site.token,
                                           'error': site.password}})
                elif self.path == '/api/fs/list':
                    self.respond({'code': site.list_code, 'data': {
                        'content': [{'name': name, 'size': len(data), 'is_dir': False}
                                    for name, data in site.files.items()],
                        'total': len(site.files),
                    }})
                elif self.path == '/api/fs/mkdir':
                    self.respond({'code': 200})
                else:
                    self.respond({'code': 404})

            def do_GET(self):
                site.events.append(('GET', self.path, None,
                                    bool(self.headers.get('Authorization'))))
                if self.path.startswith('/api/') and site.redirect_api:
                    self.send_response(307)
                    self.send_header('Location', site.redirect_api)
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
                if self.path == '/api/me':
                    self.respond({'code': 200, 'data': {'base_path': site.base_path}})
                    return
                if self.path.startswith('/download/'):
                    if site.redirect_download:
                        self.send_response(302)
                        self.send_header('Location', site.redirect_download)
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                        return
                    name = unquote(self.path.removeprefix('/download/'))
                    body = site.files[name]
                    if site.corrupt_archive_download and name.endswith('.tar.gz'):
                        body = b'x' * len(body)
                    self.send_response(200)
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.respond({'code': 404}, 404)

            def do_PUT(self):
                path = unquote(self.headers['File-Path'])
                name = path.rsplit('/', 1)[-1]
                site.events.append(('PUT', self.path, path,
                                    bool(self.headers.get('Authorization'))))
                body = self.rfile.read(int(self.headers['Content-Length']))
                if site.upload_http_code != 200:
                    self.respond({'message': site.token}, site.upload_http_code)
                elif (site.fail_upload or self.headers.get('Authorization') != site.token or
                        self.headers.get('Overwrite') != 'false' or
                        self.headers.get('As-Task') != 'true' or
                        self.headers.get('X-File-Sha256') != hashlib.sha256(body).hexdigest()):
                    self.respond({'code': 500, 'message': site.token})
                elif name in site.files:
                    self.respond({'code': 403, 'message': 'file exists'})
                else:
                    task_id = site.task_id_prefix + str(len(site.tasks) + 1)
                    site.tasks[task_id] = {'name': name, 'body': body,
                                           'states': list(site.upload_task_states)}
                    task = {} if site.omit_task_id else {'id': task_id, 'state': 0}
                    self.respond({'code': 200, 'data': {'task': task}})

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


class PublisherTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.archive = Path(self.directory.name) / 'car2_v261030-1001.tar.gz'
        self.archive.write_bytes(b'example-archive-content\n' * 4096)
        self.checksum = self.archive.with_name(self.archive.name + '.sha256')
        digest = hashlib.sha256(self.archive.read_bytes()).hexdigest()
        self.checksum.write_text(f'{digest}  {self.archive.name}\n')
        self.site = Site()
        self.addCleanup(self.site.close)
        self.client = OpenList(self.site.url, allow_local_http=True)
        self.client.login('publisher', self.site.password)

    def run_publish(self):
        with contextlib.redirect_stdout(io.StringIO()):
            publish(self.client, self.archive, self.checksum, self.site.public_directory)

    def uploads(self):
        return [event[2] for event in self.site.events if event[0] == 'PUT']

    def test_archive_is_publicly_verified_before_checksum_is_uploaded(self):
        self.run_publish()
        self.assertEqual(self.uploads(), ['/' + self.archive.name, '/' + self.checksum.name])
        archive_read = next(i for i, e in enumerate(self.site.events)
                            if e[1] == '/download/' + self.archive.name)
        checksum_write = next(i for i, e in enumerate(self.site.events)
                              if e[0] == 'PUT' and e[2].endswith('.sha256'))
        self.assertLess(archive_read, checksum_write)
        for event in self.site.events:
            if event[1] == '/api/fs/get' or event[1].startswith('/download/'):
                self.assertFalse(event[3], 'public requests must never carry the publisher token')
        self.assertEqual(self.site.files[self.archive.name], self.archive.read_bytes())
        self.assertFalse(any(e[1] == '/api/fs/mkdir' for e in self.site.events))
        self.assertEqual(self.site.public_list_payloads,
                         [{'path': self.site.public_directory}] * 3)

    def test_guest_listing_denied_prevents_any_file_upload(self):
        self.site.public_listing = False
        with self.assertRaisesRegex(PublishError, 'code 401'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_archive_hidden_from_guest_listing_prevents_checksum_upload(self):
        self.site.hidden_public_files.add(self.archive.name)
        with self.assertRaisesRegex(PublishError, 'Archive is not visible'):
            self.run_publish()
        self.assertEqual(self.uploads(), ['/' + self.archive.name])
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_hidden_checksum_cannot_report_a_discoverable_release(self):
        self.site.hidden_public_files.add(self.checksum.name)
        with self.assertRaisesRegex(PublishError, 'Release pair is not visible'):
            self.run_publish()
        self.assertIn(self.archive.name, self.site.files)
        self.assertIn(self.checksum.name, self.site.files)

    def test_admin_path_is_not_treated_as_scoped_account_path(self):
        self.site.base_path = '/'
        self.run_publish()
        self.assertEqual(self.uploads()[0], '/ELF3/bxi_arm_car2/' + self.archive.name)
        self.assertIn(('POST', '/api/fs/mkdir', '/ELF3/bxi_arm_car2', True), self.site.events)

    def test_outside_account_base_is_rejected_before_any_write(self):
        self.site.base_path = '/private'
        with self.assertRaisesRegex(PublishError, 'outside'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])
        self.assertFalse(any(e[1] == '/api/fs/mkdir' for e in self.site.events))

    def test_login_failure_never_exposes_echoed_credentials(self):
        with self.assertRaises(PublishError) as caught:
            self.client.login('publisher', 'wrong')
        self.assertNotIn(self.site.password, str(caught.exception))
        self.assertNotIn(self.site.token, str(caught.exception))
        self.assertEqual(self.uploads(), [])

    def test_upload_failure_does_not_publish_checksum_or_leak_token(self):
        self.site.fail_upload = True
        with self.assertRaises(PublishError) as caught:
            self.run_publish()
        self.assertEqual(self.uploads(), ['/' + self.archive.name])
        self.assertNotIn(self.site.token, str(caught.exception))
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_async_upload_waits_through_retries_before_public_verification(self):
        self.site.upload_task_states = [0, 1, 3, 5, 6, 8, 9, 2]
        with patch('publish_ota.UPLOAD_POLL_INTERVAL', 0):
            self.run_publish()
        self.assertEqual([state for _, state in self.site.task_polls],
                         self.site.upload_task_states * 2)
        first_download = next(i for i, event in enumerate(self.site.events)
                              if event[1] == '/api/fs/get')
        archive_polls = [i for i, event in enumerate(self.site.events)
                         if event[1] == '/api/task/upload/info?tid=upload-1']
        self.assertEqual(len(archive_polls), len(self.site.upload_task_states))
        self.assertLess(archive_polls[-1], first_download)
        self.assertIn(self.checksum.name, self.site.files)

    def test_failed_or_canceled_task_does_not_publish_checksum_or_leak_secrets(self):
        for state in (4, 7):
            with self.subTest(state=state):
                self.site.events.clear()
                self.site.upload_task_states = [1, state]
                with patch('publish_ota.UPLOAD_POLL_INTERVAL', 0):
                    with self.assertRaisesRegex(PublishError, 'failed or was canceled') as caught:
                        self.run_publish()
                self.assertEqual(self.uploads(), ['/' + self.archive.name])
                self.assertNotIn(self.checksum.name, self.site.files)
                self.assertFalse(any(e[1] == '/api/fs/get' for e in self.site.events))
                self.assertNotIn(self.site.token, str(caught.exception))
                self.assertNotIn(self.site.password, str(caught.exception))

    def test_pending_task_times_out_without_publishing_checksum(self):
        self.site.upload_task_states = [1]
        # Advance only the poller's clock; the HTTP exchange is real and retains
        # a generous socket timeout even on a loaded CI runner.
        with patch('publish_ota.time') as clock:
            clock.monotonic.side_effect = [0, 0, 0.1, 1000000]
            with self.assertRaisesRegex(PublishError, 'timed out'):
                self.run_publish()
        self.assertTrue(self.site.task_polls)
        self.assertEqual(self.uploads(), ['/' + self.archive.name])
        self.assertNotIn(self.checksum.name, self.site.files)
        self.assertFalse(any(e[1] == '/api/fs/get' for e in self.site.events))

    def test_upload_response_without_task_id_cannot_publish_checksum(self):
        self.site.omit_task_id = True
        with self.assertRaisesRegex(PublishError, 'no task ID'):
            self.run_publish()
        self.assertEqual(self.site.task_polls, [])
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_upload_gateway_timeout_is_not_treated_as_success(self):
        self.site.upload_http_code = 504
        with self.assertRaisesRegex(PublishError, 'HTTP 504') as caught:
            self.run_publish()
        self.assertEqual(self.site.task_polls, [])
        self.assertEqual(self.uploads(), ['/' + self.archive.name])
        self.assertNotIn(self.checksum.name, self.site.files)
        self.assertNotIn(self.site.token, str(caught.exception))

    def test_task_api_failure_does_not_expose_untrusted_query_or_body(self):
        self.site.task_id_prefix = self.site.token + ' & ' + self.site.password
        self.site.task_info_code = 403
        with self.assertRaisesRegex(PublishError, 'code 403') as caught:
            self.run_publish()
        self.assertTrue(self.site.task_polls, 'task IDs must be URL encoded')
        self.assertNotIn(self.site.token, str(caught.exception))
        self.assertNotIn(self.site.password, str(caught.exception))
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_corrupt_readback_prevents_publication(self):
        self.site.corrupt_archive_download = True
        with self.assertRaisesRegex(PublishError, 'content mismatch'):
            self.run_publish()
        self.assertEqual(self.uploads(), ['/' + self.archive.name])
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_private_downloads_prevent_publication(self):
        self.site.private_downloads = True
        with self.assertRaisesRegex(PublishError, 'code 403'):
            self.run_publish()
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_identical_published_files_are_verified_without_overwrite(self):
        self.site.files[self.archive.name] = self.archive.read_bytes()
        self.site.files[self.checksum.name] = self.checksum.read_bytes()
        self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_existing_unpublished_archive_can_finish_publication(self):
        self.site.files[self.archive.name] = self.archive.read_bytes()
        self.run_publish()
        self.assertEqual(self.uploads(), ['/' + self.checksum.name])

    def test_different_existing_archive_is_never_overwritten(self):
        self.site.files[self.archive.name] = b'x' * self.archive.stat().st_size
        with self.assertRaisesRegex(PublishError, 'content mismatch'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_orphan_published_checksum_is_not_repaired_implicitly(self):
        self.site.files[self.checksum.name] = self.checksum.read_bytes()
        with self.assertRaisesRegex(PublishError, 'without its archive'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_directory_permission_failure_is_not_treated_as_empty(self):
        self.site.list_code = 403
        with self.assertRaisesRegex(PublishError, 'code 403'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_bad_local_checksum_prevents_all_remote_writes(self):
        self.checksum.write_text('incorrect checksum')
        with self.assertRaisesRegex(PublishError, 'Local checksum'):
            self.run_publish()
        self.assertEqual(self.uploads(), [])

    def test_login_redirect_does_not_forward_credentials(self):
        other = Site()
        self.addCleanup(other.close)
        self.site.redirect_api = other.url + '/api/auth/login'
        with self.assertRaisesRegex(PublishError, 'HTTP 307'):
            self.client.login('publisher', self.site.password)
        self.assertEqual(other.events, [])

    def test_authenticated_api_redirect_does_not_forward_token(self):
        other = Site()
        self.addCleanup(other.close)
        self.site.redirect_api = other.url + '/api/me'
        with self.assertRaisesRegex(PublishError, 'HTTP 307'):
            self.run_publish()
        self.assertEqual(other.events, [])
        self.assertEqual(self.uploads(), [])

    def test_malformed_http_errors_do_not_expose_response_text(self):
        with patch.object(self.client.api_opener, 'open',
                          side_effect=http.client.BadStatusLine(self.site.token)):
            with self.assertRaises(PublishError) as caught:
                self.client.account_directory(self.site.public_directory)
        self.assertNotIn(self.site.token, str(caught.exception))
        self.site.files[self.archive.name] = self.archive.read_bytes()
        with patch.object(self.client.download_opener, 'open',
                          side_effect=http.client.BadStatusLine(self.site.password)):
            with self.assertRaises(PublishError) as caught:
                self.client.verify_public_file(self.archive, self.site.public_directory,
                                               hashlib.sha256(self.archive.read_bytes()).hexdigest())
        self.assertNotIn(self.site.password, str(caught.exception))

    def test_download_redirect_remains_anonymous(self):
        other = Site()
        self.addCleanup(other.close)
        other.files[self.archive.name] = self.archive.read_bytes()
        self.site.redirect_download = other.url + '/download/' + self.archive.name
        self.client.upload(self.archive, '/', hashlib.sha256(self.archive.read_bytes()).hexdigest())
        self.client.verify_public_file(self.archive, '/ELF3/bxi_arm_car2',
                                       hashlib.sha256(self.archive.read_bytes()).hexdigest())
        self.assertEqual(other.events, [('GET', '/download/' + self.archive.name, None, False)])

    def test_plaintext_or_credential_urls_are_rejected(self):
        with self.assertRaisesRegex(PublishError, 'HTTPS'):
            OpenList('http://download.bxirobotics.cn')
        with self.assertRaises(PublishError) as caught:
            OpenList('https://user:secret@example.com')
        self.assertNotIn('secret', str(caught.exception))
        self.site.raw_url = 'http://storage.invalid/archive'
        with self.assertRaisesRegex(PublishError, 'HTTPS'):
            self.run_publish()
        self.assertNotIn(self.checksum.name, self.site.files)

    def test_plaintext_download_redirect_is_rejected(self):
        self.site.redirect_download = 'http://storage.invalid/archive'
        with self.assertRaisesRegex(PublishError, 'HTTPS'):
            self.run_publish()
        self.assertNotIn(self.checksum.name, self.site.files)


if __name__ == '__main__':
    unittest.main()
