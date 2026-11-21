# syntax=docker/dockerfile:1
#
# KOMPAS ground station — headless GUI container
#
# Base: jlesage/baseimage-gui ships a virtual X server (TigerVNC Xvnc), the
# Openbox window manager, a VNC server on :5900 and a noVNC web client on
# :5800, all supervised. Tkinter windows render into that virtual display, so
# the app runs on any headless host and is reached from a browser or VNC client.
#
# Debian 12 is chosen deliberately: its system Python is 3.11, which matches
# the interpreter the project was developed on (see the cpython-311 caches).
# For reproducible builds, pin a full tag, e.g. debian-12-v4.x.y.
FROM jlesage/baseimage-gui:debian-12-v4

# ---------------------------------------------------------------------------
# Native system dependencies
# ---------------------------------------------------------------------------
# python3-tk      Tkinter + Tcl/Tk 8.6 runtime (the GUI toolkit)
# libgl1          libGL.so.1                } required at import time by the
# libglib2.0-0    libglib / libgthread      } opencv-python wheel (verified
# libsm6, libxext6, libx11-6, libxcb1       } with ldd against cv2*.so)
# fonts-dejavu-core  a readable default font for Tk and the OpenCV HUD
#
# FFmpeg is NOT installed here: the opencv-python and av wheels each bundle
# their own FFmpeg build, so VideoWriter('mp4v') and PyAV work without it.
# `add-pkg` is the base image's apt wrapper (update + install + cleanup).
RUN add-pkg \
        python3 \
        python3-venv \
        python3-tk \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libx11-6 \
        libxcb1 \
        fonts-dejavu-core

# ---------------------------------------------------------------------------
# Python environment (a venv avoids Debian's PEP 668 "externally managed" block)
# ---------------------------------------------------------------------------
ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYGAME_HIDE_SUPPORT_PROMPT=1

RUN python3 -m venv "$VIRTUAL_ENV"

# Install dependencies before copying source so code edits don't bust the cache.
# (Tkinter is not a pip package; it comes from python3-tk above.)
COPY requirements.txt /tmp/requirements.txt
RUN pip install --prefer-binary -r /tmp/requirements.txt \
 && rm /tmp/requirements.txt

# Build-time smoke test: fail the build now, not at launch, if a native
# library is missing. (Only imports — creating a Tk window needs the display.)
RUN python -c "import tkinter, cv2, numpy, PIL.ImageTk, av, yaml, serial, pygame, matplotlib; \
print('tk', tkinter.TkVersion, '| opencv', cv2.__version__, '| av', av.__version__)"

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
COPY . /app
COPY docker/startapp.sh /startapp.sh
RUN chmod 755 /startapp.sh

# Name shown in the browser tab / VNC title.
RUN set-cont-env APP_NAME "KOMPAS"

# Virtual screen size (runtime-overridable with -e). The control window is
# 800x600 and the video window opens beside it, so leave room for both.
ENV DISPLAY_WIDTH=1440 \
    DISPLAY_HEIGHT=1000

# The app reads its settings from here. On first start, startapp.sh copies the
# default config/drone_config.yaml into the /config volume, so you can edit it
# from the host (./data/drone_config.yaml with the compose file) and restart.
ENV KOMPAS_CONFIG=/config/drone_config.yaml

# 5800 = noVNC web UI, 5900 = raw VNC.
# UDP ports the app *binds* to receive from the drone:
#   8890 = telemetry/state, 11111 = video. (8889 is outbound commands only.)
EXPOSE 5800 5900 8890/udp 11111/udp

# /config is the base image's persistent, writable home. The app is started
# from there, so its relative logs/ and recordings/ directories land in it.
VOLUME ["/config"]
