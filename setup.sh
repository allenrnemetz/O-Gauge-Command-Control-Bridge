#!/bin/bash
# Setup script - run this once after extracting the ZIP
# Open a terminal in this folder and type:  bash setup.sh

cd "$(dirname "$0")"

echo "=============================================================="
echo "  Setting up Bridge installer files..."
echo "=============================================================="
echo ""

# Make all shell scripts executable
chmod +x *.sh 2>/dev/null

# Make .desktop files executable
chmod +x *.desktop 2>/dev/null

# Mark .desktop files as trusted (Cinnamon/MATE requirement)
for f in *.desktop; do
    if [ -f "$f" ]; then
        gio set "$f" metadata::trusted true 2>/dev/null || true
        echo "  Ready: $f"
    fi
done

echo ""
echo "=============================================================="
echo "  Done! You can now double-click:"
echo "    - Install Bridge  (for new installs)"
echo "    - Update Bridge   (to update an existing install)"
echo "=============================================================="
