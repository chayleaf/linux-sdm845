#!/bin/sh -e
# Copyright 2020 Oliver Smith
# SPDX-License-Identifier: GPL-3.0-or-later
# usage: install_pmbootstrap.sh [ADDITIONAL_PACKAGE, ...]

: "${PMBOOTSTRAP_TAG:="master"}"
: "${PMBOOTSTRAP_URL:="https://gitlab.com/postmarketOS/pmbootstrap.git"}"

# Add the "origin-original" remote to an existing pmaports.git clone, so
# pmbootstrap can use its channels.cfg.
add_remote_origin_original() {
	# Skip if existing (happens in single runner setup)
	remote_url="https://gitlab.com/postmarketOS/pmaports.git"
	remote_url_existing="$(git -C "$pmaports" \
				remote get-url origin-original 2>/dev/null \
				|| true)"
	if [ "$remote_url" = "$remote_url_existing" ]; then
		echo "Remote 'origin-original' is already configured"
		git -C "$pmaports" fetch -q origin-original
		return
	fi

	# Mark pmaports dir as 'safe directory', so git doesn't complain about
	# it being owned by another user.
	git config --global --add safe.directory "$pmaports"
	su pmos -c "git config --global --add safe.directory '$pmaports'"

	# Add the remote, display the output only on error
	if ! git -C "$pmaports" \
		remote add -f origin-original \
		"https://gitlab.com/postmarketOS/pmaports.git" \
		>/tmp/git_remote_add 2>&1; then
		echo "ERROR: failed to add original remote with git!"
		echo
		cat /tmp/git_remote_add
		exit 1
	fi
}

# Set up depends
depends="coreutils
	git
	openssl
	procps
	python3
	sudo
	$*"
printf "Installing dependencies:"
for i in $depends; do
	printf "%s" " $i"
done
printf "\n"
# shellcheck disable=SC2086
apk -q add $depends

# Set up binfmt_misc
if ! mount | grep -q /proc/sys/fs/binfmt_misc; then
	mount -t binfmt_misc none /proc/sys/fs/binfmt_misc
fi

# Create pmos user
if id "pmos" > /dev/null 2>&1; then
	echo "User 'pmos' exists already"
else
	echo "Creating pmos user"
	adduser -D pmos
	chown -R pmos:pmos .
	echo 'pmos ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
fi

# pmaports: either checked out in current dir, or let pmbootstrap download it
pmaports="$(cd "$(dirname "$0")"; pwd -P)"
pmaports_arg=""
if [ -e "$pmaports/pmaports.cfg" ]; then
	echo "Found pmaports.cfg in current dir"
	pmaports_arg="--aports '$pmaports'"

	add_remote_origin_original

	# Use the channel of the current branch
	mkdir -p ~pmos/.config
	( echo "[pmbootstrap]"
	  echo "is_default_channel = False" ) > ~pmos/.config/pmbootstrap.cfg
	chown -R pmos:pmos ~pmos/.config
fi

# Download pmbootstrap (to /tmp/pmbootstrap)
echo "Downloading pmbootstrap ($PMBOOTSTRAP_TAG): $PMBOOTSTRAP_URL"
cd /tmp
rm -rf pmbootstrap
git clone -q "$PMBOOTSTRAP_URL"
git -C pmbootstrap checkout -q "$PMBOOTSTRAP_TAG"

# Install to $PATH and init
ln -sf /tmp/pmbootstrap/pmbootstrap.py /usr/local/bin/pmbootstrap
echo "Initializing pmbootstrap"
if ! su pmos -c "yes '' | pmbootstrap \
		$pmaports_arg \
		--details-to-stdout \
		init \
		>/tmp/pmb_init 2>&1"; then
	cat /tmp/pmb_init
	echo
	echo "ERROR: pmbootstrap init failed!"
	echo
	echo "Most likely, this gets fixed by rebasing on master (or whatever"
	echo "branch yours is based on). Please do this and try again."
	echo
	echo "Let us know in the chat or issues if you have trouble with that"
	echo "and we will be happy to help. Sorry for the inconvenience."
	exit 1
fi
echo ""
