"""
Minimal terminal listener that toggles playback when the spacebar is pressed.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty
from typing import Callable, Optional

SPACE_KEYS = {" "}


def is_space_keypress(char: str) -> bool:
    """Return True when the provided character should toggle playback."""
    return char in SPACE_KEYS


class SpacebarListener:
    """
    Background listener that watches stdin for the spacebar key.

    When the spacebar is pressed the provided callback is invoked. The listener
    gracefully no-ops when stdin is not a TTY (common in CI environments).
    """

    def __init__(self, on_toggle: Callable[[], None], poll_timeout: float = 0.1):
        self._on_toggle = on_toggle
        self._poll_timeout = poll_timeout
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._tty_settings: Optional[list[str]] = None

    def start(self) -> bool:
        """Start the background listener. Returns False if stdin is not a TTY."""
        if self._thread is not None:
            return True

        if not sys.stdin.isatty():
            return False

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop the listener and restore terminal settings."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._tty_settings is not None and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._tty_settings)

    def _run(self) -> None:
        fd = sys.stdin.fileno()
        self._tty_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while not self._stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], self._poll_timeout)
                if sys.stdin in ready:
                    char = sys.stdin.read(1)
                    if is_space_keypress(char):
                        self._on_toggle()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._tty_settings)
