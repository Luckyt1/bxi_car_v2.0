#!/usr/bin/env bash
# Keep a public, immutable source pair for OpenList's background download.
set -euo pipefail

version=${RELEASE_VERSION:?RELEASE_VERSION is required}
repo=${GITHUB_REPOSITORY:?GITHUB_REPOSITORY is required}
commit=${GITHUB_SHA:?GITHUB_SHA is required}
[[ "$version" =~ ^v[0-9]{6}(-[1-9][0-9]*)?$ ]]
archive="car2_${version}.tar.gz"
checksum="${archive}.sha256"
(cd dist && sha256sum --check "$checksum")

temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
tag_sha="$(timeout 30 git ls-remote "${GITHUB_SERVER_URL:-https://github.com}/$repo.git" \
  "refs/tags/$version" "refs/tags/$version^{}" |
  awk '$2 ~ /\^\{\}$/ { peeled=$1 } { plain=$1 } END { print peeled ? peeled : plain }')"
if [[ -n "$tag_sha" && "$tag_sha" != "$commit" ]]; then
  echo '::error::The release tag already points to a different commit.'
  exit 1
fi

# Listing also finds an interrupted draft; transport/API failures stop here.
release_ids="$(gh api --paginate "repos/$repo/releases?per_page=100" \
  --jq ".[] | select(.tag_name == \"$version\") | .id")"
if [[ "$release_ids" == *$'\n'* ]]; then
  echo '::error::Multiple releases use this version; refusing to choose one.'
  exit 1
fi
if [[ -z "$release_ids" ]]; then
  printf 'Tested car2 OTA package built from commit %s.\n' "$commit" > "$temporary/notes"
  gh release create "$version" --repo "$repo" --target "$commit" \
    --title "car2 $version" --notes-file "$temporary/notes" --draft --latest=false
fi
gh release view "$version" --repo "$repo" \
  --json targetCommitish,assets,isDraft > "$temporary/release.json"
python3 - "$temporary/release.json" "$commit" <<'PY'
import json
import sys
release = json.load(open(sys.argv[1], encoding='utf-8'))
if release['targetCommitish'] != sys.argv[2]:
    raise SystemExit('Release target does not match the tested commit')
PY

for file in "$archive" "$checksum"; do
  if python3 - "$temporary/release.json" "$file" <<'PY'
import json
import sys
release = json.load(open(sys.argv[1], encoding='utf-8'))
sys.exit(0 if any(asset['name'] == sys.argv[2] for asset in release['assets']) else 1)
PY
  then
    gh release download "$version" --repo "$repo" --pattern "$file" --dir "$temporary"
    if ! cmp -s "dist/$file" "$temporary/$file"; then
      echo "::error::Existing release asset differs from the tested file: $file"
      exit 1
    fi
  else
    gh release upload "$version" "dist/$file" --repo "$repo"
  fi
done

if [[ "$(gh release view "$version" --repo "$repo" --json isDraft --jq .isDraft)" == true ]]; then
  gh release edit "$version" --repo "$repo" --draft=false --latest=false
fi
resolved_commit="$(gh api "repos/$repo/commits/$version" --jq .sha)"
if [[ "$resolved_commit" != "$commit" ]]; then
  echo '::error::Published release tag does not match the tested commit.'
  exit 1
fi
echo "Public GitHub release verified: $version"
