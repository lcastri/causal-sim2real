#!/usr/bin/env bash
# Simple helper to convert selected MP4s to optimized GIFs using ffmpeg.
# Usage: chmod +x scripts/convert_videos_to_gifs.sh && ./scripts/convert_videos_to_gifs.sh

set -euo pipefail

OUT_DIR="static/gifs"
mkdir -p "$OUT_DIR"

# Parameters for conversion
FPS=12
WIDTH=480

convert() {
  local in="$1"
  local out="$2"
  local palette="$out-palette.png"

  echo "Converting $in -> $out (fps=$FPS width=$WIDTH)"
  ffmpeg -y -i "$in" -vf "fps=${FPS},scale=${WIDTH}:-1:flags=lanczos,palettegen" "$palette"
  ffmpeg -y -i "$in" -i "$palette" -lavfi "fps=${FPS},scale=${WIDTH}:-1:flags=lanczos [x]; [x][1:v] paletteuse" "$out"
  rm -f "$palette"
}

convert "static/videos/S1-inference-noncausal-screen.mp4" "$OUT_DIR/S1-inference-noncausal-screen.gif"
convert "static/videos/S1-inference-causal-screen.mp4"     "$OUT_DIR/S1-inference-causal-screen.gif"
convert "static/videos/S2-inference-noncausal-screen.mp4" "$OUT_DIR/S2-inference-noncausal-screen.gif"
convert "static/videos/S2-inference-causal-screen.mp4"     "$OUT_DIR/S2-inference-causal-screen.gif"

echo "Done. Generated GIFs in $OUT_DIR"
