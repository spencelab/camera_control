# camera_control
GUI for control of cameras, triggerboxes, treadmills, and subsequent processing and uploading.

# TODO
1. Add "Select System" button left of "Launch System"
2. Add "Configure Camera" button for cameras that may be launched and detected but not configured? Start seems to handle anways, but the camera state stays at "unconfig". Fix in cbrng?
3. Add required "User (eg Ajay, gms, ajs, gtk, joshross, jaqr)" field to metadata. Append to session folder
4. Add dummy file in camera session folder with user name, rat, trial, etc? something so you know who owns folders/files. Silly? can handle after upload? No, useful, so know who needs to upload. Or unk

# Processing rules:
```
cam1          -> camera dir cam1 on host cam1, using ssh
cam1@cam1     -> camera dir cam1 on host cam1, using ssh
cam1@local    -> camera dir cam1 on this same machine, no ssh
cam1=local    -> same as cam1@local
cam1@ros2test -> camera dir cam1 on host ros2test
```


# Session telemetry bags

Starting a camera recording now starts a controller-local ROS 2 bag first. The
bag is stored under:

```text
~/camera_sessions/<session>/rosbag/telemetry_001/
```

Rig policy lives in `configs/rigs.yaml`:

```yaml
telemetry:
  treadmill: required   # required | optional | off
  triggerbox: required  # required | optional | off
  camera_events: true
```

The recorder writes MCAP when `rosbag2_storage_mcap` is installed, captures
Chrony status at start/stop, finalizes on Stop Recording, and creates
`TELEMETRY_COMPLETE` or `TELEMETRY_INCOMPLETE` markers.

Install the MCAP plugin if needed:

```bash
sudo apt install ros-$ROS_DISTRO-rosbag2-storage-mcap
```

After stopping a treadmill recording, generate the quick sanity plot:

```bash
cd ~/ros2_ws/src/camera_control
python3 tools/plot_treadmill_bag.py ~/camera_sessions/<session> --show
```

The script writes `rosbag/treadmill_speed_sanity.png` and CSV. If no physical
speed reports were present, it says so explicitly while still plotting the
commanded HIIT profile.

# Multi-camera synchronization audit

The Processing tab has an **Audit multi-cam sync** action. The same analysis is
automatically and safely attempted after per-camera processing for **Process
raws**, **Info + audit**, **Process + verify**, and **Process + verify + upload**.
A failed/incomplete multi-camera audit is recorded in `processing_manifest.tsv`
but does not abort later overnight pipeline attempts.

Each camera's raw audit uses the acquisition cadence stored with that recording:

1. `effective_settings.camera.expected_hardware_fps` when hardware trigger is on
2. `effective_settings.camera.fps`
3. the same keys under `requested_settings`

The `conversion.fps` value in `processing*.yaml` is **MP4 playback FPS only**.
For example, a 100 Hz acquisition transcoded at 10 fps is a 10x slow-motion
movie, but its raw trigger audit is still performed at 100 Hz.

Session-level outputs are written under:

```text
~/camera_sessions/<session>/multicam_sync/
  multicam_sync_report.txt
  multicam_sync_summary.yaml
  multicam_sync_alignment.csv
  inputs/<cam>/...audit.csv + metadata...
  MULTICAM_SYNC_PASS
  # or MULTICAM_SYNC_PASS_WITH_EXCLUSIONS / FAIL / INCOMPLETE
```

`multicam_sync_alignment.csv` contains the shared global trigger, synchronized
frame index, each camera's MP4/raw frame index, frame number and timestamps, plus
`all_cameras_valid`. Missing frames therefore remain explicitly addressable
without accumulating movie de-sync.

The tool can also be run directly on a consolidated session directory:

```bash
cd ~/ros2_ws/src/camera_control
python3 camera_control/multicam_sync_audit.py ~/camera_sessions/<session>
```

On the acquisition/master computer, it can collect the audit inputs directly
from camera hosts first:

```bash
python3 camera_control/multicam_sync_audit.py \
  ~/camera_sessions/<session> \
  --camera cam1@cam1 \
  --camera cam2@cam2 \
  --camera cam3@cam3 \
  --camera cam4@cam4 \
  --collect
```

PASS and PASS WITH EXCLUSIONS return shell exit code 0. FAIL returns 1 and
incomplete/missing-input audits return 2.

## Disposable-session cleanup

The Processing tab also has a red **DELETE SELECTED SESSIONS** action for test
runs that should never be processed or uploaded. After a prominent destructive
confirmation, it removes the selected session directory from tmill and every
configured camera host. The upload/storage server is never touched. The delete
uses strict path guards, treats already-absent remote copies as success, and
keeps the tmill session if any configured camera-host delete fails so the
operation remains visible and retryable.
