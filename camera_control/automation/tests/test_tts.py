# Author: Andrew England (andrewengland19)
"""Unit tests for the zero-dependency Speaker. No audio is produced: system
backends are exercised with a monkeypatched ``subprocess.Popen`` and the backend
is injected, so these run anywhere (no espeak/say required)."""

from automation import tts
from automation.tts import Speaker


class _RecordingPopen:
    """Stand-in for subprocess.Popen that records argv and never spawns."""

    calls = []

    def __init__(self, argv, **kwargs):
        type(self).calls.append(argv)
        self.argv = argv

    def poll(self):
        return 0  # already finished

    def terminate(self):
        pass


def test_no_backend_logs_and_never_raises():
    logs = []
    spk = Speaker(_backend=None, log_fn=logs.append)
    assert spk.available() is False
    spk.say("hello")  # must not raise
    assert any("no backend" in m for m in logs)


def test_muted_does_not_call_backend(monkeypatch):
    _RecordingPopen.calls = []
    monkeypatch.setattr(tts.subprocess, "Popen", _RecordingPopen)
    logs = []
    spk = Speaker(_backend="say", muted=True, log_fn=logs.append)
    spk.say("hi")
    assert _RecordingPopen.calls == []
    assert any("muted" in m for m in logs)


def test_say_backend_launches_nonblocking(monkeypatch):
    _RecordingPopen.calls = []
    monkeypatch.setattr(tts.subprocess, "Popen", _RecordingPopen)
    spk = Speaker(_backend="say")
    spk.say("Camera recording started")
    assert _RecordingPopen.calls == [["say", "Camera recording started"]]


def test_empty_text_is_ignored(monkeypatch):
    _RecordingPopen.calls = []
    monkeypatch.setattr(tts.subprocess, "Popen", _RecordingPopen)
    Speaker(_backend="say").say("")
    assert _RecordingPopen.calls == []


def test_argv_for_each_backend():
    assert Speaker._argv_for("say", "x") == ["say", "x"]
    assert Speaker._argv_for("spd-say", "x") == ["spd-say", "-e", "x"]
    assert Speaker._argv_for("espeak", "x") == ["espeak", "x"]
    assert Speaker._argv_for("espeak-ng", "x") == ["espeak-ng", "x"]


def test_detect_prefers_pyttsx3_when_importable(monkeypatch):
    monkeypatch.setattr(Speaker, "_pyttsx3_importable", staticmethod(lambda: True))
    spk = Speaker()
    assert spk.backend == "pyttsx3"


def test_detect_falls_through_to_system_when_no_pyttsx3(monkeypatch):
    monkeypatch.setattr(Speaker, "_pyttsx3_importable", staticmethod(lambda: False))
    # Force a known system command to be "present".
    monkeypatch.setattr(tts.shutil, "which", lambda cmd: "/usr/bin/say" if cmd == "say" else None)
    monkeypatch.setattr(tts.sys, "platform", "darwin")
    spk = Speaker()
    assert spk.backend == "say"


def test_detect_none_when_nothing_available(monkeypatch):
    monkeypatch.setattr(Speaker, "_pyttsx3_importable", staticmethod(lambda: False))
    monkeypatch.setattr(tts.shutil, "which", lambda cmd: None)
    spk = Speaker()
    assert spk.backend is None
    assert spk.available() is False


def test_backend_failure_is_swallowed(monkeypatch):
    def _boom(argv, **kwargs):
        raise OSError("no such binary")

    monkeypatch.setattr(tts.subprocess, "Popen", _boom)
    logs = []
    spk = Speaker(_backend="espeak", log_fn=logs.append)
    spk.say("hi")  # must not raise
    assert any("failed" in m.lower() for m in logs)
