#!/usr/bin/env bash
# Fetch a build input into the current directory.
# Usage: fetch_cached.sh <file name> <upstream url>
# 1. Take the copy stored in this repository's "build-cache" release, if present.
# 2. Otherwise download from upstream, retrying network errors and 5xx answers.
# 3. On push builds, store a freshly downloaded file in the build-cache release,
#    so a later outage of the upstream host no longer breaks the build.
set -u
name="$1"
url="$2"
repo="${GITHUB_REPOSITORY:?}"

if gh release download build-cache --repo "$repo" --pattern "$name" --clobber > /dev/null 2>&1 && [ -s "$name" ]; then
    echo "$name: taken from build-cache"
    exit 0
fi

rm -f "$name"
if ! wget -q --tries=6 --waitretry=30 --retry-connrefused --retry-on-http-error=429,500,502,503,504 \
        --timeout=120 -O "$name" "$url"; then
    rm -f "$name"
    echo "::error::download of $name failed: $url"
    exit 1
fi
echo "$name: downloaded from upstream"

if [ "${GITHUB_EVENT_NAME:-}" = "push" ]; then
    gh release upload build-cache --repo "$repo" --clobber "$name" > /dev/null 2>&1 \
        && echo "$name: stored in build-cache" \
        || echo "::warning::$name could not be stored in build-cache"
fi
exit 0
