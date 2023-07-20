#!/bin/sh -e
# Build and test the ramdisk for a given device

if [ "$(id -u)" = 0 ]; then
	set -x
	# Install cdba!
	curl --location --output /tmp/cdba.apk --location \
		https://gitlab.com/postmarketOS/pmaports/-/jobs/5179200037/artifacts/raw/packages/edge/x86_64/cdba-0.1_git20230923-r0.apk
	apk add --allow-untrusted /tmp/cdba.apk
	exec su "${TESTUSER:-pmos}" -c "sh -e $0 $1"
fi

echo "Current user is $GITLAB_USER_NAME ($GITLAB_USER_LOGIN)"

# CDBA_DEVICE comes from environment
CDBA_HOST="vault.connolly.tech"
CDBA_PORT="2233"

mkdir -p ~/.ssh
cat > ~/.ssh/config <<EOF
Host tiger.cdba
	User cdba
	Port $CDBA_PORT
	HostName $CDBA_HOST
	StrictHostKeyChecking no
EOF

chown -R "$(id -u):$(id -g)" ~/.ssh
chmod 700 ~/.ssh

eval $(ssh-agent -s)
echo "$CDBA_SSH_KEY" | tr -d '\r' | ssh-add -
ls -l ~/.ssh

BOOTIMG=/tmp/"$DEVICE"-boot.img

echo "Booting $BOOTIMG on $CDBA_DEVICE"

# Timeout after 30 seconds of inactivity
# Power off board and quit if the board prints 20 '~' characters (-c 0)
cdba -T 30 -c 0 -h tiger.cdba -b $CDBA_DEVICE $BOOTIMG | tee /tmp/cdba.log

cp /tmp/cdba.log cdba.log

if grep -q "PMOS-CI-OK" /tmp/cdba.log; then
	echo "All tests pass"
else
	exit 1
fi
