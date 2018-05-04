#!/bin/bash


for f in *.{jpg,jpeg,JPG,JPEG,png,PNG}; do
  [ -f "$f" ] || continue
  magick "$f" \
    -colorspace sRGB \
    -resize 1280x1280\> \
    -unsharp 0x0.75+0.75+0.008 \
    -strip \
    -quality 85 \
    -define webp:auto-filter=true \
    "${f%.*}.webp"
done