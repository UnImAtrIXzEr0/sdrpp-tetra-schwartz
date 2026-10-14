#!/bin/bash
# Build an AppImage from an SDR++ Brown build directory.
# Usage: ./make_appimage.sh <build_dir> [linuxdeploy]
set -e

BUILD_DIR="$(readlink -f "${1:-build}")"
LINUXDEPLOY="${2:-linuxdeploy}"
SRC_DIR="$(dirname "$(readlink -f "$0")")"
APPDIR="$(pwd)/AppDir"

rm -rf "$APPDIR"
DESTDIR="$APPDIR" cmake --install "$BUILD_DIR" --prefix /usr

# Licence and origin notices (GPL-3.0, AGPL-3.0 for osmo-tetra, NOTICE)
mkdir -p "$APPDIR/usr/share/doc/sdrpp-tetra-schwartz"
cp "$SRC_DIR/license" "$APPDIR/usr/share/doc/sdrpp-tetra-schwartz/LICENSE-GPL-3.0.txt"
cp "$SRC_DIR/decoder_modules/ch_tetra_demodulator/src/decoder/COPYING" "$APPDIR/usr/share/doc/sdrpp-tetra-schwartz/LICENSE-AGPL-3.0-osmo-tetra.txt"
cp "$SRC_DIR/NOTICE.md" "$APPDIR/usr/share/doc/sdrpp-tetra-schwartz/NOTICE.md"

# Desktop entry and icon with AppImage-friendly names
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"
cp "$SRC_DIR/root/res/icons/sdrpp.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/sdrpp.png"
cat > "$APPDIR/usr/share/applications/sdrpp.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Terminal=false
Exec=sdrpp
Name=SDR++ TETRA-SCHWARTZ
Comment=Amateur radio TETRA edition based on SDR++Brown (experimental beta)
Icon=sdrpp
Categories=HamRadio;Network;
EOF

# Launcher: separate config root (paths differ from a normal install) and
# SDRPP_APPDIR so the core resolves modules/resources inside the mount.
cat > "$(pwd)/AppRun.sh" <<'EOF'
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
export SDRPP_APPDIR="$HERE"
export LD_LIBRARY_PATH="$HERE/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for a in "$@"; do
    case "$a" in -r|--root|-r=*|--root=*) exec "$HERE/usr/bin/sdrpp" "$@";; esac
done
ROOT="${XDG_CONFIG_HOME:-$HOME/.config}/sdrpp_brown_appimage"
mkdir -p "$ROOT"
exec "$HERE/usr/bin/sdrpp" -r "$ROOT" "$@"
EOF
chmod +x "$(pwd)/AppRun.sh"

# Collect dependencies of the binary, the core and every plugin
PLUGIN_ARGS=()
for p in "$APPDIR"/usr/lib/sdrpp/plugins/*.so; do
    PLUGIN_ARGS+=(--deploy-deps-only "$p")
done

export NO_STRIP=1
export LD_LIBRARY_PATH="$APPDIR/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LDAI_OUTPUT="SDRPP-TETRA-SCHWARTZ${TETRA_VERSION:+-v$TETRA_VERSION}-beta-x86_64.AppImage"
"$LINUXDEPLOY" --appimage-extract-and-run \
    --appdir "$APPDIR" \
    --executable "$APPDIR/usr/bin/sdrpp" \
    --library "$APPDIR/usr/lib/libsdrpp_core.so" \
    "${PLUGIN_ARGS[@]}" \
    --desktop-file "$APPDIR/usr/share/applications/sdrpp.desktop" \
    --icon-file "$APPDIR/usr/share/icons/hicolor/256x256/apps/sdrpp.png" \
    --custom-apprun "$(pwd)/AppRun.sh" \
    --output appimage
