# Author: Andrew England (andrewengland19)
"""Speaker — a lightweight, non-blocking, zero-dependency text-to-speech helper.

Design goals (see plan):
  * Never block the Qt/ROS thread. Every utterance is fire-and-forget: system
    backends are launched with ``subprocess.Popen`` (returns immediately) and the
    optional ``pyttsx3`` backend runs on a daemon worker thread.
  * Add no required dependency. Backends are probed at construction, in order:
        pyttsx3 (only if already importable) -> macOS ``say`` -> Linux ``spd-say``
        -> ``espeak`` / ``espeak-ng`` -> silent no-op (log only).
    If nothing is available the Speaker degrades to logging the phrase, so the
    automation still runs on a headless / audio-less machine.
  * Be mutable at runtime (``muted``) so the operator can silence voice from the UI.

This module imports nothing from PySide6/rclpy and is safe to import anywhere.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import threading
from typing import Any, Callable, List, Optional

# Sentinel so callers can force backend=None (no-op) distinctly from "auto-detect".
_AUTODETECT = object()


class Speaker:
    """Speak short status phrases without blocking the caller."""

    def __init__(
        self,
        enabled: bool = True,
        muted: bool = False,
        log_fn: Optional[Callable[[str], None]] = None,
        prefer_pyttsx3: bool = True,
        _backend: Any = _AUTODETECT,
    ) -> None:
        self.enabled = enabled
        self.muted = muted
        self._log_fn = log_fn
        # Keep references to spawned Popen objects so we can reap them and, on
        # shutdown, avoid orphaning; capped to avoid unbounded growth.
        self._procs: List[subprocess.Popen] = []
        # Backend is one of: "pyttsx3", "say", "spd-say", "espeak",
        # "espeak-ng", or None (no-op). Injectable for tests via ``_backend``.
        self.backend: Optional[str] = (
            self._detect_backend(prefer_pyttsx3) if _backend is _AUTODETECT else _backend
        )

    # ------------------------------------------------------------------ detect
    @staticmethod
    def _pyttsx3_importable() -> bool:
        try:
            import pyttsx3  # noqa: F401
            return True
        except Exception:
            return False

    def _detect_backend(self, prefer_pyttsx3: bool) -> Optional[str]:
        if prefer_pyttsx3 and self._pyttsx3_importable():
            return "pyttsx3"
        # macOS
        if sys.platform == "darwin" and shutil.which("say"):
            return "say"
        # Linux / other: prefer speech-dispatcher, then espeak variants.
        for cmd in ("spd-say", "espeak-ng", "espeak"):
            if shutil.which(cmd):
                return cmd
        # Fall back to `say` even off-darwin if somehow present.
        if shutil.which("say"):
            return "say"
        return None

    def available(self) -> bool:
        """True if a real audio backend was found (not the log-only fallback)."""
        return self.backend is not None

    # -------------------------------------------------------------------- log
    def _log(self, msg: str) -> None:
        if self._log_fn is not None:
            self._log_fn(msg)

    # -------------------------------------------------------------------- say
    def say(self, text: str) -> None:
        """Speak ``text`` asynchronously. Safe to call from the GUI thread."""
        if not text:
            return
        if not self.enabled or self.muted:
            self._log(f"TTS (muted): {text}")
            return
        if self.backend is None:
            # No audio backend — surface the phrase in the log so it isn't lost.
            self._log(f"TTS (no backend): {text}")
            return
        try:
            self._speak_via_backend(text)
        except Exception as exc:  # never let TTS raise into the automation
            self._log(f"TTS failed ({self.backend}): {exc} -- {text}")

    def _speak_via_backend(self, text: str) -> None:
        if self.backend == "pyttsx3":
            self._speak_pyttsx3(text)
            return
        argv = self._argv_for(self.backend, text)
        # Popen returns immediately; discard child output. This never blocks.
        proc = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
        )
        self._reap()
        self._procs.append(proc)

    @staticmethod
    def _argv_for(backend: str, text: str) -> List[str]:
        if backend == "say":
            return ["say", text]
        if backend == "spd-say":
            # -e: exit after speaking; keeps invocations independent.
            return ["spd-say", "-e", text]
        if backend in ("espeak", "espeak-ng"):
            return [backend, text]
        raise ValueError(f"unknown TTS backend: {backend}")

    def _speak_pyttsx3(self, text: str) -> None:
        def _worker() -> None:
            try:
                import pyttsx3
                engine = pyttsx3.init()
                engine.say(text)
                engine.runAndWait()
                engine.stop()
            except Exception as exc:
                self._log(f"TTS pyttsx3 worker failed: {exc} -- {text}")

        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------------ reap
    def _reap(self) -> None:
        """Drop finished processes so the list can't grow without bound."""
        still: List[subprocess.Popen] = []
        for p in self._procs:
            if p.poll() is None:
                still.append(p)
        self._procs = still

    def shutdown(self) -> None:
        """Best-effort: stop pending speech (called on app close)."""
        for p in self._procs:
            try:
                if p.poll() is None:
                    p.terminate()
            except Exception:
                pass
        self._procs = []
