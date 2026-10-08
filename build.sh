#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v psp-config >/dev/null || ! command -v psp-gcc >/dev/null; then
  echo 'PSPSDK psp-config and psp-gcc required; run in configured Ubuntu/WSL' >&2
  exit 1
fi

make -C "$root/psp" clean
make -C "$root/psp"

prx="$root/psp/lcdc_capture.prx"
if ! strings "$prx" | grep -q 'PSP_LCDC_CAPTURE_20261008'; then
  echo 'Wrong PRX build signature' >&2
  exit 1
fi
python3 "$root/tests/check_freeze.py"

# Build a ready-to-use release package without changing the frozen PSP binary.
dist="$root/dist"
rm -rf "$dist"
mkdir -p "$dist/PSP-LCDC-Capture/pc"
cp "$prx" "$dist/lcdc_capture.prx"
cp "$prx" "$dist/PSP-LCDC-Capture/lcdc_capture.prx"
cp "$root/README.md" "$dist/PSP-LCDC-Capture/README.md"
cp "$root/pc/bridge.py" "$dist/PSP-LCDC-Capture/pc/bridge.py"
cp "$root/pc/obs.html" "$dist/PSP-LCDC-Capture/pc/obs.html"
cp "$root/pc/PSP_LCDC_Capture_Start.bat" "$dist/PSP-LCDC-Capture/pc/PSP_LCDC_Capture_Start.bat"

ROOT="$root" python3 - <<'PY'
import os, zipfile
from pathlib import Path
root = Path(os.environ['ROOT'])
dist = root / 'dist'
base = dist / 'PSP-LCDC-Capture'
out = dist / 'PSP-LCDC-Capture.zip'
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
    for p in sorted(base.rglob('*')):
        if p.is_file():
            z.write(p, p.relative_to(dist))
PY

echo 'PSP LCDC Capture'
echo "PRX:     $dist/lcdc_capture.prx"
echo "Release: $dist/PSP-LCDC-Capture.zip"
