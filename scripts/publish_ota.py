#!/usr/bin/env python3
"""Publish a verified car2 archive to OpenList; the checksum is committed last."""

import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import time
from urllib import error, parse, request


UPLOAD_TASK_TIMEOUT = 15 * 60
UPLOAD_POLL_INTERVAL = 3


class PublishError(RuntimeError):
    pass


class NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # API requests carry credentials. Never forward them to another URL.
        return None


class DownloadRedirect(request.HTTPRedirectHandler):
    def __init__(self, validate_url):
        self.validate_url = validate_url

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        self.validate_url(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def absolute_directory(value):
    if (not isinstance(value, str) or not value.startswith('/') or
            any(part == '..' for part in value.split('/')) or
            any(ord(char) < 32 for char in value)):
        raise PublishError('OpenList directories must be absolute paths without traversal')
    return PurePosixPath(value)


class OpenList:
    def __init__(self, site, *, allow_local_http=False):
        # The HTTP exception exists only for localhost unit tests; the CLI does
        # not expose it. Production API and download traffic require HTTPS.
        self.allow_local_http = allow_local_http
        self.validate_url(site)
        parts = parse.urlsplit(site)
        if parts.query or parts.fragment:
            raise PublishError('The OpenList site URL cannot contain a query or fragment')
        self.site = site.rstrip('/')
        self.token = ''
        self.api_opener = request.build_opener(NoRedirect())
        self.download_opener = request.build_opener(DownloadRedirect(self.validate_url))

    def validate_url(self, url):
        try:
            parts = parse.urlsplit(url)
            allowed = parts.scheme == 'https' or (
                self.allow_local_http and parts.scheme == 'http' and
                parts.hostname in ('127.0.0.1', '::1', 'localhost'))
            if not allowed or not parts.hostname or parts.username or parts.password:
                raise ValueError()
            _ = parts.port
        except (TypeError, ValueError):
            raise PublishError('OpenList and download URLs must use HTTPS without credentials') from None

    def api(self, method, endpoint, payload=None, *, authenticated=True, body=None,
            headers=None, timeout=300):
        outgoing = dict(headers or {})
        if authenticated:
            if not self.token:
                raise PublishError('OpenList login is required')
            outgoing['Authorization'] = self.token
        if payload is not None:
            body = json.dumps(payload).encode('utf-8')
            outgoing['Content-Type'] = 'application/json'
        req = request.Request(self.site + endpoint, data=body, headers=outgoing, method=method)
        # Query values can originate in the response (for example a task ID).
        # Include only the fixed API path in diagnostics, never those values.
        endpoint_name = endpoint.partition('?')[0]
        try:
            with self.api_opener.open(req, timeout=timeout) as response:
                encoded = response.read(2 * 1024 * 1024 + 1)
            if len(encoded) > 2 * 1024 * 1024:
                raise PublishError('OpenList API response exceeded the size limit')
            result = json.loads(encoded)
        except error.HTTPError as exc:
            raise PublishError(f'OpenList {endpoint_name} failed (HTTP {exc.code})') from None
        except (OSError, error.URLError, http.client.HTTPException, ValueError):
            # Remote error bodies and signed URLs can contain credentials.
            raise PublishError(f'OpenList {endpoint_name} failed or returned invalid JSON') from None
        if not isinstance(result, dict) or result.get('code') != 200:
            code = result.get('code') if isinstance(result, dict) else None
            code = code if isinstance(code, int) else 'invalid'
            raise PublishError(f'OpenList {endpoint_name} failed (code {code})')
        return result.get('data')

    def login(self, username, password):
        if not username or not password:
            raise PublishError('Set both OTA_USERNAME and OTA_PASSWORD repository secrets')
        data = self.api('POST', '/api/auth/login', {
            'username': username, 'password': password,
        }, authenticated=False)
        token = data.get('token') if isinstance(data, dict) else None
        if not isinstance(token, str) or not token:
            raise PublishError('OpenList login returned no token')
        self.token = token

    def account_directory(self, public_directory):
        user = self.api('GET', '/api/me')
        if not isinstance(user, dict):
            raise PublishError('OpenList returned invalid account information')
        base = absolute_directory(user.get('base_path'))
        public = absolute_directory(public_directory)
        try:
            relative = public.relative_to(base)
        except ValueError:
            raise PublishError('OTA directory is outside the publisher account base path') from None
        return '/' if str(relative) == '.' else '/' + str(relative)

    def list_files(self, directory, *, authenticated=True):
        # Anonymous discovery must use the robot's exact request. In particular,
        # refresh can require an account permission that guests do not possess.
        payload = {'path': directory}
        if authenticated:
            payload.update(page=1, per_page=0, refresh=True)
        data = self.api('POST', '/api/fs/list', payload, authenticated=authenticated)
        if not isinstance(data, dict):
            raise PublishError('OpenList returned an invalid directory listing')
        entries = data.get('content') or []
        if (not isinstance(entries, list) or
                data.get('total', len(entries)) != len(entries)):
            raise PublishError('OpenList directory listing is incomplete')
        return {entry['name'] for entry in entries
                if isinstance(entry, dict) and isinstance(entry.get('name'), str)
                and not entry.get('is_dir')}

    def upload(self, file, directory, digest):
        remote = str(PurePosixPath(directory) / file.name)
        with file.open('rb') as stream:
            data = self.api('PUT', '/api/fs/put', body=stream, headers={
                'File-Path': parse.quote(remote, safe='/'),
                'Content-Type': 'application/octet-stream',
                'Content-Length': str(file.stat().st_size),
                'X-File-Sha256': digest,
                'As-Task': 'true',
                'Overwrite': 'false',
            })
        task = data.get('task') if isinstance(data, dict) else None
        task_id = task.get('id') if isinstance(task, dict) else None
        if not isinstance(task_id, str) or not task_id:
            raise PublishError('OpenList upload returned no task ID')
        self.wait_for_upload(task_id)

    def wait_for_upload(self, task_id):
        endpoint = '/api/task/upload/info?' + parse.urlencode({'tid': task_id})
        deadline = time.monotonic() + UPLOAD_TASK_TIMEOUT
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PublishError('OpenList upload task timed out')
            task = self.api('POST', endpoint, timeout=min(60, remaining))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PublishError('OpenList upload task timed out')
            state = task.get('state') if isinstance(task, dict) else None
            if type(state) is not int or state not in range(10):
                raise PublishError('OpenList returned an invalid upload task state')
            # OpenList/tache: 2 succeeds; 4 is canceled; 7 is terminal failure.
            # Retrying tasks can have an error and 100% progress before success.
            if state == 2:
                return
            if state in (4, 7):
                raise PublishError('OpenList upload task failed or was canceled')
            time.sleep(min(UPLOAD_POLL_INTERVAL, remaining))

    def verify_public_file(self, file, public_directory, digest):
        # Robots use this unauthenticated API. Checking only with the publisher
        # account could publish a release that no robot is allowed to download.
        metadata = self.api('POST', '/api/fs/get', {
            'path': str(PurePosixPath(public_directory) / file.name),
        }, authenticated=False)
        expected_size = file.stat().st_size
        if (not isinstance(metadata, dict) or metadata.get('is_dir') or
                metadata.get('size') != expected_size):
            raise PublishError(f'Public file size mismatch: {file.name}')
        url = metadata.get('raw_url')
        self.validate_url(url)
        actual_hash = hashlib.sha256()
        received = 0
        try:
            # A separate opener and a fresh request prevent Authorization from
            # reaching a storage provider or its redirect/CDN destination.
            with self.download_opener.open(request.Request(url), timeout=300) as response:
                while block := response.read(1024 * 1024):
                    received += len(block)
                    if received > expected_size:
                        raise PublishError(f'Public file size mismatch: {file.name}')
                    actual_hash.update(block)
        except (OSError, error.URLError, http.client.HTTPException):
            raise PublishError(f'Public download verification failed: {file.name}') from None
        if received != expected_size or actual_hash.hexdigest() != digest:
            raise PublishError(f'Public file content mismatch: {file.name}')


def publish(client, archive, checksum, public_directory):
    if not re.fullmatch(r'car2_v\d{6}(?:-[1-9]\d*)?\.tar\.gz', archive.name):
        raise PublishError('Archive name must be car2_vYYMMDD[-N].tar.gz')
    if checksum.name != archive.name + '.sha256':
        raise PublishError('Checksum filename must match the archive')
    if any(not path.is_file() or path.is_symlink() for path in (archive, checksum)):
        raise PublishError('Archive and checksum must be regular, non-symlink files')
    digest = file_hash(archive)
    expected = f'{digest}  {archive.name}\n'.encode('ascii')
    if checksum.stat().st_size != len(expected) or checksum.read_bytes() != expected:
        raise PublishError('Local checksum does not match the archive')
    checksum_digest = hashlib.sha256(expected).hexdigest()
    public_directory = str(absolute_directory(public_directory))
    directory = client.account_directory(public_directory)
    if directory != '/':
        # OpenList mkdir is idempotent for existing directories. A restricted
        # account's base directory must be provisioned by the administrator.
        client.api('POST', '/api/fs/mkdir', {'path': directory})
    existing = client.list_files(directory)
    # Check directory access before uploading anything. Public raw downloads
    # alone do not prove the robot can discover a release in its catalog.
    client.list_files(public_directory, authenticated=False)
    if checksum.name in existing and archive.name not in existing:
        raise PublishError('Published checksum exists without its archive; refusing to replace it')
    if archive.name not in existing:
        client.upload(archive, directory, digest)
    client.verify_public_file(archive, public_directory, digest)
    if archive.name not in client.list_files(public_directory, authenticated=False):
        raise PublishError('Archive is not visible in the anonymous OTA directory listing')
    if checksum.name not in existing:
        # This is the publication point: robots discover only archive+checksum
        # pairs, and the archive has already passed an anonymous full readback.
        client.upload(checksum, directory, checksum_digest)
    client.verify_public_file(checksum, public_directory, checksum_digest)
    public_files = client.list_files(public_directory, authenticated=False)
    if not {archive.name, checksum.name}.issubset(public_files):
        raise PublishError('Release pair is not visible in the anonymous OTA directory listing')
    print(f'OTA release verified: {public_directory}/{archive.name}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--checksum', type=Path, required=True)
    parser.add_argument('--site', default='https://download.bxirobotics.cn')
    parser.add_argument('--remote-dir', default='/ELF3/bxi_arm_car2', help='Public, site-wide directory')
    args = parser.parse_args()
    try:
        client = OpenList(args.site)
        client.login(os.environ.get('OTA_USERNAME', ''), os.environ.get('OTA_PASSWORD', ''))
        publish(client, args.archive, args.checksum, args.remote_dir)
    except (PublishError, OSError) as exc:
        print(f'OTA publish failed: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
