# easy_handeye2_demo

## How to start 

### Eye-in-hand

```bash
ros2 launch easy_handeye2_demo calibrate.launch.py calibration_type:=eye_in_hand tracking_base_frame:=tr_base tracking_marker_frame:=tr_marker robot_base_frame:=panda_link0 robot_effector_frame:=panda_link8 freehand_robot_movement:=true
```

### Eye-on-base

```bash
ros2 launch easy_handeye2_demo calibrate.launch.py calibration_type:=eye_on_base tracking_base_frame:=tr_base tracking_marker_frame:=tr_marker robot_base_frame:=panda_link0 robot_effector_frame:=panda_link8 freehand_robot_movement:=true
```