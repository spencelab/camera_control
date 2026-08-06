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
