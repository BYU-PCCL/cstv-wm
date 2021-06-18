# Script example usage


## Transform publishers

```sh
python3 src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py world cam_1_depth_optical_frame --file '_cams_set_info_world_to_1'
python3 src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py world cam_2_depth_optical_frame --file '_cams_set_info_world_to_2'
python3 src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py world cam_4_depth_optical_frame --file '_cams_set_info_world_to_4'
```

## Realsense nodes

```sh
./just_restart.sh ros2 launch realsense2_camera rs_launch.py initial_reset:=true enable_pointcloud:=true base_depth_frame:=cam_1_link depth_optical_frame_id:=cam_1_depth_optical_frame serial_no:=f0221612 camera_name:=cam_1
./just_restart.sh ros2 launch realsense2_camera rs_launch.py initial_reset:=true enable_pointcloud:=true base_depth_frame:=cam_2_link depth_optical_frame_id:=cam_2_depth_optical_frame serial_no:=f0271635 camera_name:=cam_2
./just_restart.sh ros2 launch realsense2_camera rs_launch.py initial_reset:=true enable_pointcloud:=true base_depth_frame:=cam_4_link depth_optical_frame_id:=cam_4_depth_optical_frame serial_no:=f0246222 camera_name:=cam_4
```

## Multi-cloud publisher

```sh
ros2 run multi_cloud cloud_publisher --ros-args -p 'point_cloud_topics:=["/cam_1/depth/color/points", "/cam_2/depth/color/points", "cam_4/depth/color/points"]'
```

## Cloud flattener

```sh
ros2 run cloud_flattener slice_publisher --ros-args -p 'cloud_topic:="/multi_cloud"'
```

## TODO:

- Add example for initializing and adjusting transforms
