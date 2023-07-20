#!/bin/sh -e
# Fetches artifacts from a previous job to avoid rebuilding the kernel while testing

URL=$1

if [ -z "$URL" ]; then
	echo "Usage: $0 <url>"
	exit 1
fi

mkdir -p packages/edge/aarch64/ || true

wget -P packages/edge/aarch64/ "$URL"
wget -P packages/edge/aarch64/ $(dirname "$URL")/APKINDEX.tar.gz

exit 0
