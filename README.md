# easy_handeye2_demo

## How to start 

### Calibrating

```bash
ros2 launch easy_handeye2_demo calibrate.launch.py calibration_type:=eye_in_hand tracking_base_frame:=tr_base tracking_marker_frame:=tr_marker robot_base_frame:=panda_link0 robot_effector_frame:=panda_link8
```

### Checking the result

```bash
ros2 launch easy_handeye2_demo check_calibration.launch.py calibration_type:=eye_in_hand tracking_base_frame:=tr_base tracking_marker_frame:=tr_marker robot_base_frame:=panda_link0 robot_effector_frame:=panda_link8
```