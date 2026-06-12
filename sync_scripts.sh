#!/bin/bash

# KSP destination directory in Windows (mounted in WSL)
DEST_DIR="/mnt/d/SteamLibrary/steamapps/common/Kerbal Space Program/Ships/Script"

# Create the destination directory if it doesn't exist
mkdir -p "$DEST_DIR"

echo "Synchronizing folders to Kerbal Space Program..."

# Copy directories
cp -r Boot/ "$DEST_DIR/"
cp -r lib/ "$DEST_DIR/"
cp -r missions/ "$DEST_DIR/"

echo "----------------------------------------"
echo "Synchronization completed successfully!"
echo "Files copied to: $DEST_DIR"
echo "----------------------------------------"
