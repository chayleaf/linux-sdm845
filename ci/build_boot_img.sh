#!/bin/sh -ex

# XXX: pass in needed vars rather than this awful hack
# sudo -E preserves environment (including these variables from root user lol)
USER=pmos
HOME=/home/$USER

pmb() {
	pmbootstrap --details-to-stdout $@
}

ROOTFS="/home/pmos/.local/var/pmbootstrap/chroot_rootfs_$DEVICE"

pmb config device $DEVICE

# Fetch additional packages from a pmaports MR
if [ -n "$ADDITIONAL_PKGS_JOB" ]; then
	PROJECT_ID="8065375" # postmarketOS/pmaports
	PRIVATE_TOKEN="$SDM845_CI_TOKEN"
	. ci/gitlab_api.sh

	# Sometimes the download randomly fails in CI for fun reasons
	# so worst case it's better to retry a few times than fail
	# the job...
	TRIES=5
	for i in $(seq 1 $TRIES); do
		if [ $i -gt 1 ]; then
			echo "Retrying apk $@"
		fi
		glapi_fetch_artifacts $ADDITIONAL_PKGS_JOB /tmp/artifacts.$ADDITIONAL_PKGS_JOB.zip
		TMP=$(mktemp -d)

		unzip /tmp/artifacts.$ADDITIONAL_PKGS_JOB.zip -d $TMP/ && break
		sleep 3
		done
	ls -R $TMP/
	sudo cp $TMP/packages/edge/aarch64/*.apk /home/pmos/.local/var/pmbootstrap/packages/edge/aarch64/
fi
pmb index

echo "Exporting initial boot.img"
# We add unl0kr because we need SOMETHING that provides postmarketos-fde-unlocker
# and osk-sdl is HUUGE
pmb chroot -r apk add unl0kr

echo "Adding CI hook"
pmb chroot -r apk add postmarketos-mkinitfs-hook-ci
pmb export

cp $ROOTFS/boot/boot.img /tmp/$DEVICE-boot.img
