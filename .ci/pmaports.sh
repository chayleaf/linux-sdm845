#!/bin/sh

set -ex
PKG="$1"
TAG="$2"
BRANCH="$(basename $PKG)-$TAG"

grev() {
	git rev-parse --verify $1
}

pushd /home/pmos/.local/var/pmbootstrap/cache_git/pmaports

git config --local user.name "SDM845 CI"
git config --local user.email "sdm845-ci@blah.com"

if [ "$(grev $BRANCH || true)" != "" ]; then
	echo "Branch $BRANCH already exists, ammending"
	git stash push . || true
	git checkout $BRANCH
	git stash pop || true
	git add -A
	git commit --amend --no-edit
	echo "Amended pmaports commit:"
else
	git add -A
	git commit -m "$PKG: Upgrade to $TAG"
	echo "Created pmaports commit:"
fi
git show -1
git remote remove ssh_origin || true # local repo state may be cached
git remote add ssh_origin "git@$CI_SERVER_HOST:$CI_PROJECT_PATH.git"
eval $(ssh-agent -s)
LEN=$(echo "$SSH_PUSH_KEY" | wc)
echo "SSH key length $LEN"
echo "$SSH_PUSH_KEY" | tr -d '\r' | ssh-add -
BRANCH="$(basename $PKG)-$TAG"
git checkout -b "$BRANCH"
GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no" git push --force ssh_origin -o ci.skip HEAD:$BRANCH

PRIVATE_TOKEN="$SDM845_CI_TOKEN"
# pmaports project ID
PROJECT_ID="8065375"
. .ci/gitlab_api.sh

glapi_open_mr "$BRANCH" "$PKG: Upgrade to $TAG" "Cc: @sdm845-mainline @calebccff"
