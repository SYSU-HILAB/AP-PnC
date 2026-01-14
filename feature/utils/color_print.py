"""ANSI color helpers for consistent colorful logging in feature utilities."""

from __future__ import annotations


class Colors:
    """ANSI color codes for terminal output."""

    GREEN = "\033[92m"  # Success green
    YELLOW = "\033[93m"  # Warning yellow
    BLUE = "\033[94m"  # Info blue
    RED = "\033[91m"  # Error red
    BOLD = "\033[1m"  # Bold text
    RESET = "\033[0m"  # Reset formatting
    CHECKMARK = "\u2713"  # ✓ symbol
    CROSS = "\u2717"  # ✗ symbol


def color_print(text: str, color: str = Colors.RESET) -> None:
    """Print text with ANSI color formatting."""
    print(f"{color}{text}{Colors.RESET}")


__all__ = ["Colors", "color_print"]
