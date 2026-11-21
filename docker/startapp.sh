#!/bin/sh
# Launched by the base image's supervisor as the unprivileged app user,
# with DISPLAY already pointing at the virtual X server.
set -e

# Work from the writable, persistent /config volume: the app creates its
# logs/ and recordings/ folders relative to the current directory, and
# /app is read-only for the app user.
cd /config

# Seed an editable config on first run; never overwrite the user's edits.
if [ ! -f "$KOMPAS_CONFIG" ]; then
    cp /app/config/drone_config.yaml "$KOMPAS_CONFIG"
fi

exec python /app/main.py
