#!/usr/bin/env bash
set -euo pipefail

install -D -m 0644 trail.py "$STAGE$APP_INSTALL_DIR/trail.py"
# Lets the gpio group switch the Cap's 5 V rail (same rule M5Stack's LoRa app uses)
install -D -m 0644 packaging/71-pocket-trail-ext5v.rules "$STAGE/etc/udev/rules.d/71-pocket-trail-ext5v.rules"

tmp=$(mktemp)
cat >"$tmp" <<'LAUNCHER'
#!/bin/sh
exec python3 /usr/share/APPLaunch/apps/pocket-trail/trail.py "$@"
LAUNCHER
install -D -m 0755 "$tmp" "$STAGE$INSTALL_PREFIX/bin/pocket-trail"
rm -f "$tmp"
