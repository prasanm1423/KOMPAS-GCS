#!/usr/bin/env python3
"""
pytest configuration for KOMPAS tests.

Provides a cv2 stub fixture for environments without OpenCV, and ensures
proper isolation between tests that need real cv2 and those that don't.
"""

import sys
from unittest.mock import MagicMock

import pytest


class _FakeCV2:
    """Minimal cv2 stub for tests that don't need real OpenCV."""
    def __getattr__(self, name):
        return None
    def __call__(self, *a, **kw):
        return None


@pytest.fixture(scope="session", autouse=True)
def cv2_stub_for_no_cv2_env():
    """
    Provide a cv2 stub for tests that import modules which transitively
    require cv2, but only when real cv2 is not available.

    This fixture is session-scoped and autouse, so it runs once per test session.
    It only applies the stub if cv2 is not already importable.
    """
    # Check if real cv2 is available
    cv2_available = False
    try:
        import cv2
        cv2_available = True
    except ImportError:
        pass

    if cv2_available:
        # Real cv2 is available, no stub needed
        yield
        return

    # No real cv2 - apply stub for the session
    stub = _FakeCV2()
    original_cv2 = sys.modules.get('cv2')
    original_submodules = {}
    for submodule in ['cv2.aruco', 'cv2.cuda', 'cv2.detail', 'cv2.dnn',
                      'cv2.line_descriptor', 'cv2.optflow', 'cv2.ocl',
                      'cv2.ogl', 'cv2.sfm', 'cv2.xfeatures2d']:
        original_submodules[submodule] = sys.modules.get(submodule)

    sys.modules['cv2'] = stub
    for submodule in original_submodules:
        sys.modules[submodule] = stub

    try:
        yield
    finally:
        # Restore original state
        if original_cv2 is not None:
            sys.modules['cv2'] = original_cv2
        else:
            sys.modules.pop('cv2', None)
        for submodule, original in original_submodules.items():
            if original is not None:
                sys.modules[submodule] = original
            else:
                sys.modules.pop(submodule, None)


# Alternative: per-test isolation fixture for tests that explicitly need cv2 isolation
@pytest.fixture
def isolated_cv2():
    """
    Fixture that provides an isolated cv2 stub for a single test.
    Use this when a test needs to ensure cv2 is stubbed regardless of environment.
    """
    stub = _FakeCV2()
    original_cv2 = sys.modules.get('cv2')
    original_submodules = {}
    for submodule in ['cv2.aruco', 'cv2.cuda', 'cv2.detail', 'cv2.dnn',
                      'cv2.line_descriptor', 'cv2.optflow', 'cv2.ocl',
                      'cv2.ogl', 'cv2.sfm', 'cv2.xfeatures2d']:
        original_submodules[submodule] = sys.modules.get(submodule)

    sys.modules['cv2'] = stub
    for submodule in original_submodules:
        sys.modules[submodule] = stub

    try:
        yield stub
    finally:
        if original_cv2 is not None:
            sys.modules['cv2'] = original_cv2
        else:
            sys.modules.pop('cv2', None)
        for submodule, original in original_submodules.items():
            if original is not None:
                sys.modules[submodule] = original
            else:
                sys.modules.pop(submodule, None)