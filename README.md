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

