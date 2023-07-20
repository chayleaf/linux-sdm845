#!/bin/sh -ex
# Install a key from an APKINDEX.tar.gz file

INDEX=$1
NAME=$2

if [ -z "$INDEX" ] || [ -z "$NAME" ]; then
	echo "Usage: $0 <index> <key_name>"
	exit 1
fi

TMP=$(mktemp -d)

tar -xzf $INDEX -C $TMP/
# config_apk_keys gets mounted to /etc/apk/keys in the chroot
sudo cp $TMP/.*.pub /home/pmos/.local/var/pmbootstrap/config_apk_keys/$NAME.rsa.pub
