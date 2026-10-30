#!/bin/bash
# Generate the lantern's voice clips with macOS text-to-speech, laid out for the DFPlayer Mini:
#   sd_card/01/001.wav .. 021.wav   "Ring at 0 .. 100 percent charge" (5 % steps)
#   sd_card/02/001.wav  oath (paste the text into OATH below)
#   sd_card/02/002.wav  "Ring fully charged"
#   sd_card/02/003.wav  "Charging"
#   sd_card/02/004.wav  "Ring connected"
#   sd_card/02/005.wav  "Warning. Ring power low"
# Copy the two folders to the root of a FAT32 microSD.   usage: ./make_voice_clips.sh [voice]
# List voices: say -v '?'
set -e
VOICE=${1:-Daniel}
OUT="$(dirname "$0")/../../../ring/lantern/sd_card"
OATH="PASTE THE OATH TEXT HERE"
mkdir -p "$OUT/01" "$OUT/02"
clip() { say -v "$VOICE" -r 165 --data-format=LEI16@22050 -o "$1" "$2"; }
for i in $(seq 0 20); do clip "$OUT/01/$(printf %03d $((i + 1))).wav" "Ring at $((i * 5)) percent charge."; done
clip "$OUT/02/001.wav" "$OATH"
clip "$OUT/02/002.wav" "Ring fully charged."
clip "$OUT/02/003.wav" "Charging."
clip "$OUT/02/004.wav" "Ring connected."
clip "$OUT/02/005.wav" "Warning. Ring power low."
ls "$OUT"/01 | wc -l | xargs echo "level clips:"; ls "$OUT"/02
