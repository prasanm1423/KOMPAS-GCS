"""
Synthetic forward camera for the simulated drone.

Draws a simple but motion-faithful scene, so the video pipeline and HUD
can be exercised without hardware:
  - the horizon moves with pitch and tilts with roll
  - ground lines scroll with forward travel and spread with altitude
  - a sun marker sweeps across the sky as the aircraft yaws

Every frame is watermarked "SIMULATED FEED" so recordings and snapshots
can never be mistaken for real footage.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

WIDTH, HEIGHT = 960, 720
HFOV_DEG = 82.0                                  # horizontal field of view
FOCAL_PX = (WIDTH / 2) / math.tan(math.radians(HFOV_DEG / 2))
PX_PER_DEG = FOCAL_PX * math.pi / 180.0          # small-angle approximation
CAMERA_HEIGHT_ON_GROUND = 8.0                    # cm
LINE_SPACING = 60.0                              # cm between ground lines
SUN_AZIMUTH = 120.0                              # deg
SUN_ELEVATION = 18.0                             # deg

SKY_TOP = np.array([200, 150, 90], dtype=np.float32)      # BGR
SKY_HORIZON = np.array([235, 215, 185], dtype=np.float32)
GROUND = (70, 120, 85)
GROUND_LINE = (95, 150, 110)

# Render on a canvas large enough to rotate for roll without exposing corners.
_CANVAS = int(math.ceil(math.hypot(WIDTH, HEIGHT))) + 8


def _sky(canvas: np.ndarray, horizon_y: int):
    rows = np.clip(np.arange(_CANVAS, dtype=np.float32) / max(horizon_y, 1), 0.0, 1.0)
    gradient = SKY_TOP[None, :] * (1 - rows[:, None]) + SKY_HORIZON[None, :] * rows[:, None]
    canvas[:] = gradient[:, None, :].astype(np.uint8)


def render_frame(state) -> np.ndarray:
    """Render one BGR frame (HEIGHT x WIDTH) from a SimState snapshot."""
    canvas = np.empty((_CANVAS, _CANVAS, 3), dtype=np.uint8)
    cx = cy = _CANVAS // 2

    # Nose down (negative pitch) shows more ground, so the horizon rises.
    horizon_y = int(round(cy + state.pitch * PX_PER_DEG))
    _sky(canvas, horizon_y)
    cv2.rectangle(canvas, (0, max(horizon_y, 0)), (_CANVAS, _CANVAS), GROUND, -1)

    # Sun: fixed in the world, so yawing sweeps it across the view.
    rel = (SUN_AZIMUTH - state.yaw + 180.0) % 360.0 - 180.0
    if abs(rel) < HFOV_DEG:
        sun = (int(cx + rel * PX_PER_DEG), int(horizon_y - SUN_ELEVATION * PX_PER_DEG))
        cv2.circle(canvas, sun, 34, (170, 235, 255), -1, cv2.LINE_AA)

    # Ground lines at fixed world spacing along the direction of travel.
    # A line d cm ahead appears FOCAL * h / d pixels below the horizon.
    height = max(state.z, 0.0) + CAMERA_HEIGHT_ON_GROUND
    yaw = math.radians(state.yaw)
    travelled = state.x * math.sin(yaw) + state.y * math.cos(yaw)
    offset = travelled % LINE_SPACING
    for k in range(1, 60):
        distance = k * LINE_SPACING - offset
        if distance <= 5:
            continue
        y = int(horizon_y + FOCAL_PX * height / distance)
        if y >= _CANVAS:
            continue
        thickness = 1 if distance > 400 else 2
        cv2.line(canvas, (0, y), (_CANVAS, y), GROUND_LINE, thickness, cv2.LINE_AA)

    # Roll: rotate the whole scene about the image centre, then crop.
    if abs(state.roll) > 0.05:
        rotation = cv2.getRotationMatrix2D((cx, cy), state.roll, 1.0)
        canvas = cv2.warpAffine(canvas, rotation, (_CANVAS, _CANVAS),
                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

    frame = canvas[cy - HEIGHT // 2: cy + HEIGHT // 2, cx - WIDTH // 2: cx + WIDTH // 2].copy()

    label = "SIMULATED FEED"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    x, y = WIDTH - tw - 16, HEIGHT - 16
    cv2.rectangle(frame, (x - 6, y - th - 6), (x + tw + 6, y + 6), (0, 0, 0), -1)
    cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)
    return frame
