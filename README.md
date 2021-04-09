# aruco-detector
This is the ROS2 wrapper for Aruco detection based on ORBSLAM2 library.

Note that the path where the list of "good" Aruco codes is: `/usr/local/share/ORB_SLAM2/` and the file must be named `arucoCodes.dat`.

Here follows and example of that file:
```
23
25
```

Means that the aruco codes that will be considered by the system are only those corresponding to `23` and `25`, all the other detected codes will be discarded.
