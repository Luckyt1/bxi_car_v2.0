"""Run the real release shell helper against isolated local CLI substitutes."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


HELPER = Path(__file__).with_name('publish_github_release.sh')
VERSION = 'v261031-5001'
COMMIT = 'a' * 40
ARCHIVE = f'car2_{VERSION}.tar.gz'
CHECKSUM = ARCHIVE + '.sha256'


FAKE_CLI = r'''#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

state_path = Path(os.environ['RELEASE_TEST_STATE'])
state = json.loads(state_path.read_text())
args = sys.argv[1:]
version = os.environ['RELEASE_VERSION']
repo = os.environ['GITHUB_REPOSITORY']
commit = os.environ['GITHUB_SHA']
program = Path(sys.argv[0]).name

def save():
    state_path.write_text(json.dumps(state))

def fail(message):
    print(message, file=sys.stderr)
    raise SystemExit(1)

def option(name):
    return args[args.index(name) + 1]

state['calls'].append([program] + args)
save()
if program == 'git':
    if args != ['ls-remote', 'https://github.invalid/' + repo + '.git',
                'refs/tags/' + version, 'refs/tags/' + version + '^{}']:
        fail('Unsupported offline git request')
    if state['tag_sha']:
        print(state['tag_sha'] + '\trefs/tags/' + version)
    raise SystemExit(0)

if args[0] == 'api':
    endpoint = next(arg for arg in args if arg.startswith('repos/'))
    if endpoint == 'repos/' + repo + '/releases?per_page=100':
        if state['fail_listing']:
            fail('Simulated GitHub API transport failure')
        if state['release']:
            print(state['release']['id'])
    elif endpoint == 'repos/' + repo + '/commits/' + version:
        if not state['tag_sha']:
            fail('Release has not produced a tag')
        print(state['tag_sha'])
    else:
        fail('Unsupported offline GitHub API request')
    raise SystemExit(0)

if args[0] != 'release' or args[2] != version or option('--repo') != repo:
    fail('Unsupported offline release request')
command = args[1]
release = state['release']
if command == 'create':
    if release or '--draft' not in args:
        fail('Cannot replace a release or publish before uploading assets')
    release = {'id': 123, 'targetCommitish': option('--target'),
               'isDraft': True, 'assets': {}}
    state['release'] = release
    state['effects'].append(['create'])
elif not release:
    fail('Release does not exist')
elif command == 'view':
    if '--jq' in args:
        if option('--jq') != '.isDraft':
            fail('Unsupported release field selection')
        print(json.dumps(release['isDraft']))
    else:
        print(json.dumps({
            'targetCommitish': release['targetCommitish'],
            'isDraft': release['isDraft'],
            'assets': [{'name': name} for name in release['assets']],
        }))
elif command == 'download':
    name = option('--pattern')
    if name not in release['assets']:
        fail('Asset does not exist')
    (Path(option('--dir')) / name).write_bytes(bytes.fromhex(release['assets'][name]))
elif command == 'upload':
    path = Path(args[3])
    if '--clobber' in args or path.name in release['assets']:
        fail('Asset replacement is forbidden')
    release['assets'][path.name] = path.read_bytes().hex()
    state['effects'].append(['upload', path.name])
elif command == 'edit':
    expected = {'car2_' + version + '.tar.gz', 'car2_' + version + '.tar.gz.sha256'}
    if '--draft=false' not in args or set(release['assets']) != expected:
        fail('Both source assets must exist before publication')
    release['isDraft'] = False
    state['tag_sha'] = state['tag_sha'] or release['targetCommitish']
    state['effects'].append(['publish'])
else:
    fail('Unsupported offline release command')
save()
'''


class GithubReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        for command in ('gh', 'git'):
            script = self.bin / command
            script.write_text(FAKE_CLI)
            script.chmod(0o755)
        (self.root / 'dist').mkdir()
        archive = b'cloud-built archive bytes\x00\xff\n'
        checksum = f'{hashlib.sha256(archive).hexdigest()}  {ARCHIVE}\n'.encode()
        self.files = {ARCHIVE: archive, CHECKSUM: checksum}
        for name, content in self.files.items():
            (self.root / 'dist' / name).write_bytes(content)
        self.state_path = self.root / 'state.json'
        self.write_state({'release': None, 'tag_sha': None,
                          'fail_listing': False, 'calls': [], 'effects': []})

    def write_state(self, state):
        self.state_path.write_text(json.dumps(state))

    def state(self):
        return json.loads(self.state_path.read_text())

    def seed_draft(self, assets):
        state = self.state()
        state['release'] = {
            'id': 123, 'targetCommitish': COMMIT, 'isDraft': True,
            'assets': {name: content.hex() for name, content in assets.items()},
        }
        self.write_state(state)

    def run_helper(self):
        # No inherited GitHub credentials, user config, or network clients.
        environment = {
            'PATH': str(self.bin) + os.pathsep + os.defpath,
            'HOME': str(self.root),
            'TMPDIR': str(self.root),
            'GITHUB_REPOSITORY': 'test-owner/test-repo',
            'GITHUB_SERVER_URL': 'https://github.invalid',
            'GITHUB_SHA': COMMIT,
            'RELEASE_VERSION': VERSION,
            'RELEASE_TEST_STATE': str(self.state_path),
        }
        return subprocess.run(
            ['bash', str(HELPER)], cwd=self.root, env=environment,
            text=True, capture_output=True, timeout=10,
        )

    def assert_succeeded(self, result):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_first_publish_uploads_both_verified_files_before_publication(self):
        self.assert_succeeded(self.run_helper())
        state = self.state()
        self.assertEqual(state['release']['assets'],
                         {name: data.hex() for name, data in self.files.items()})
        self.assertEqual(state['effects'], [
            ['create'], ['upload', ARCHIVE], ['upload', CHECKSUM], ['publish'],
        ])
        self.assertFalse(state['release']['isDraft'])
        self.assertEqual(state['tag_sha'], COMMIT)

    def test_repeated_publish_keeps_existing_assets_without_another_upload(self):
        self.assert_succeeded(self.run_helper())
        state = self.state()
        state['effects'] = []
        self.write_state(state)
        self.assert_succeeded(self.run_helper())
        self.assertEqual(self.state()['effects'], [])
        self.assertEqual(self.state()['release'], state['release'])

    def test_partial_draft_resumes_only_the_missing_checksum(self):
        self.seed_draft({ARCHIVE: self.files[ARCHIVE]})
        self.assert_succeeded(self.run_helper())
        state = self.state()
        self.assertEqual(state['effects'], [['upload', CHECKSUM], ['publish']])
        self.assertEqual(state['release']['assets'][ARCHIVE], self.files[ARCHIVE].hex())
        self.assertFalse(state['release']['isDraft'])

    def test_different_existing_bytes_are_not_replaced_or_published(self):
        self.seed_draft({ARCHIVE: b'different archive'})
        before = self.state()['release']
        result = self.run_helper()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Existing release asset differs', result.stdout)
        self.assertEqual(self.state()['release'], before)
        self.assertEqual(self.state()['effects'], [])

    def test_tag_for_another_commit_is_rejected_before_creating_a_release(self):
        state = self.state()
        state['tag_sha'] = 'b' * 40
        self.write_state(state)
        result = self.run_helper()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('different commit', result.stdout)
        self.assertIsNone(self.state()['release'])
        self.assertEqual(self.state()['effects'], [])

    def test_api_listing_failure_is_not_mistaken_for_an_absent_release(self):
        state = self.state()
        state['fail_listing'] = True
        self.write_state(state)
        result = self.run_helper()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Simulated GitHub API transport failure', result.stderr)
        self.assertIsNone(self.state()['release'])
        self.assertEqual(self.state()['effects'], [])


if __name__ == '__main__':
    unittest.main()
