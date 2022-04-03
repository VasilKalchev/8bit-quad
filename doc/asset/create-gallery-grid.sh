#!/bin/bash


magick montage \
  *.webp \
  -background none \
  -geometry 400x300+5+5 \
  -tile 4x \
  gallery-grid.webp
