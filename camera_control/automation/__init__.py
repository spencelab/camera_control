# Author: Andrew England (andrewengland19)
"""Auto-Run automation package (additive).

A single-click "Autostart & Automated Protocol" workflow that chains the existing
manual controls — reset HIIT, confirm metadata, start recording (await ROS
confirmation), run the HIIT protocol, and stop recording when the belt reaches
0 cm/s — with non-blocking spoken (TTS) state callouts.

Pure/Qt only (no rclpy): the ROS node and the manual panels are injected, so the
controller and panel import and test headlessly under the GUI venv, mirroring the
``hiit`` subpackage.
"""

from .tts import Speaker  # noqa: F401
