#!/usr/bin/env bash
set -euo pipefail

REMOTE="${1:-origin}"
BASE_BRANCH="${2:-master}"
FEATURE_BRANCH="${3:-keep/viewpoint-override-clean}"
FEATURE_PATCH="${4:-/home/snow/firestorm-patches/0001-viewpoint-override-clean-feature.patch}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Run this script from inside the repository."
    exit 1
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
    echo "Working tree has tracked changes. Commit or stash changes first."
    exit 1
fi

echo "Fetching $REMOTE..."
git fetch "$REMOTE"

if git show-ref --verify --quiet "refs/heads/$FEATURE_BRANCH"; then
    echo "Rebasing $FEATURE_BRANCH onto $REMOTE/$BASE_BRANCH..."
    git checkout "$FEATURE_BRANCH"
    if ! git rebase "$REMOTE/$BASE_BRANCH"; then
        echo
        echo "Rebase stopped due to conflicts."
        echo "Resolve conflicts, then run: git rebase --continue"
        exit 1
    fi
else
    echo "Creating $FEATURE_BRANCH from $REMOTE/$BASE_BRANCH..."
    git checkout -b "$FEATURE_BRANCH" "$REMOTE/$BASE_BRANCH"
    if [[ ! -f "$FEATURE_PATCH" ]]; then
        echo "Patch not found: $FEATURE_PATCH"
        echo "Pass patch path as 4th argument."
        exit 1
    fi

    echo "Applying feature patch..."
    if ! git am "$FEATURE_PATCH"; then
        echo
        echo "Patch apply failed. Resolve manually, then use:"
        echo "  git am --continue"
        echo "or abort with:"
        echo "  git am --abort"
        exit 1
    fi
fi

echo
echo "Update complete."
git status -sb
git log --oneline --decorate -n 3
