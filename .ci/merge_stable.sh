#!/bin/sh -ex


# XXX: pass in needed vars rather than this awful hack
# sudo -E preserves environment
USER=pmos
HOME=/home/$USER

git config --global user.name "SDM845 CI"
git config --global user.email "sdm845-ci@blah.com"

RELEASES=$(curl --silent https://www.kernel.org/releases.json)

get_release() {
	REL=$1
	if [ -z "$REL" ]; then
		echo "No release specified"
		exit 1
	fi
	echo $RELEASES | jq -r ".releases | [.[] | select(.version|contains(\"$REL\"))][0]"
}

do_one_patch() {
	local URL
	URL=$1
	echo "Downloading $URL"
	curl --silent $URL | xz -d | patch -N -p1
	git add -A
	git commit -m "Merge $STABLE_VER"
}

# If current release is x.y.0 then get the whole patch
do_full_patch() {
	local STABLE_VER
	local URL
	STABLE_VER=$1
	echo "Doing full patch for $STABLE_VER"
	URL=$(get_release $STABLE_VER | jq -r ".patch.full")
	do_one_patch $URL
}

do_incremental_patches() {
	local STABLE_VER
	local URL
	local KVER
	KVER=$1
	STABLE_VER=$2
	STABLE_PATCHLEVEL=$(echo $STABLE_VER | cut -d . -f3)
	KVER_MAJOR=$(echo $KVER | cut -d . -f1)
	KVER_MINOR=$(echo $KVER | cut -d . -f2)
	KVER_PATCHLEVEL=$(echo $KVER | cut -d . -f3)
	if [ $KVER_PATCHLEVEL -eq 0 ]; then
		KVER_PATCHLEVEL=1
	fi
	for v in $(seq $KVER_PATCHLEVEL $(($STABLE_PATCHLEVEL - 1))); do
		NEXT=$(($v + 1))
		URL="https://cdn.kernel.org/pub/linux/kernel/v${KVER_MAJOR}.x/incr/patch-${KVER_MAJOR}.${KVER_MINOR}.${v}-${NEXT}.xz"
		echo "Applying incremental patch $KVER_MAJOR.$KVER_MINOR.$v -> $KVER_MAJOR.$KVER_MINOR.$NEXT"
		do_one_patch $URL
	done
}

KVER=$(make kernelversion)
KVER_MAJMIN=$(echo $KVER | cut -d . -f1,2)
LATEST_STABLE=$(echo $RELEASES | jq -r ".releases | [.[] | select(.version|contains(\"$KVER_MAJMIN\"))][0].version")
KVER_PATCHLEVEL=$(echo $KVER | cut -d . -f3)
LATEST_PATCHLEVEL=$(echo $LATEST_STABLE | cut -d . -f3)

echo "Current version $(make kernelversion), latest stable $LATEST_STABLE"

if [ "$(make kernelversion)" == "$LATEST_STABLE" ]; then
	echo "Already on latest stable";
	exit 1;
fi

if [ $KVER_PATCHLEVEL -eq 0 ]; then
	do_full_patch $LATEST_STABLE
else
	do_incremental_patches $KVER $LATEST_STABLE
fi

git tag -a sdm845-$LATEST_STABLE -m "Merged stable $LATEST_STABLE"

git remote remove ssh_origin || true # local repo state may be cached
git remote add ssh_origin "git@$CI_SERVER_HOST:$CI_PROJECT_PATH.git"
eval $(ssh-agent -s)
LEN=$(echo "$SSH_PUSH_KEY" | wc)
echo "SSH key length $LEN"
echo "$SSH_PUSH_KEY" | tr -d '\r' | ssh-add -
# Only push the tag, not the branch
GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no" git push --tags ssh_origin

echo "sdm845-$LATEST_STABLE" > latest_stable_tag
