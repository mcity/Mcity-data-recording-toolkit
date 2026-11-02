#!/bin/bash

# Installs (or with --remove, uninstalls) a GNOME autostart entry that runs
# start-cameras.sh when the mcity user logs into the desktop.
# A desktop session is required: the drivers open gnome-terminal tabs.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
desktop_file="$HOME/.config/autostart/mcity-cameras.desktop"

if [ "$1" == "--remove" ]; then
    rm -f "$desktop_file"
    echo "Removed $desktop_file"
    exit 0
fi

mkdir -p "$(dirname "$desktop_file")"
cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=Mcity cameras
Comment=LUCID sync-trigger drivers + JPEG republisher
Exec=bash $script_dir/start-cameras.sh
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=15
Terminal=false
EOF
echo "Installed $desktop_file"
