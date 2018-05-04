#!/bin/bash
# Builds the Boards Manager archive from the platform, vendored libraries
# included. Prints the checksum and size to put in
# package_8bit-quad_index.json.

set -e

here=$(dirname "$(readlink -f "$0")")
repo=$(dirname "$here")
# platform.txt is CRLF, so strip the carriage return
version=$(grep '^version=' "$here/8bit-quad/avr/platform.txt" | cut -d= -f2 | tr -d '\r')
name=8bit-quad-avr-$version
out=$repo/build

rm -rf "${out:?}/$name" "$out/$name.tar.gz"
mkdir -p "$out/$name"
cp -r "$here/8bit-quad/avr/." "$out/$name/"
tar czf "$out/$name.tar.gz" -C "$out" "$name"

echo "$name.tar.gz"
echo "  checksum: SHA-256:$(sha256sum "$out/$name.tar.gz" | cut -d' ' -f1)"
echo "  size:     $(stat -c%s "$out/$name.tar.gz")"
