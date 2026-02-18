# PC2Image - LiDAR to Image Projection Package

This ROS 2 package projects LiDAR point clouds onto RGB camera images.

## Package Structure

```
pc2image/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── pc2image
├── launch/
│   └── pointcloud_projector.launch.py
├── config/
│   └── (optional configuration files)
└── pc2image/
    ├── __init__.py
    └── PointCloudProjector.py
```

## Prerequisites

Make sure you have the following dependencies installed:

```bash
# Install Python dependencies
pip3 install scipy opencv-python numpy

# Install ROS 2 dependencies (if not already installed)
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs
```

## Building the Package

1. Copy the `pc2image` folder to your ROS 2 workspace `src` directory:
   ```bash
   cp -r pc2image ~/ros2_ws/src/
   ```

2. Navigate to your workspace root:
   ```bash
   cd ~/ros2_ws
   ```

3. Build the package:
   ```bash
   colcon build --packages-select pc2image
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Package

### Option 1: Using ROS 2 run command
```bash
ros2 run pc2image pointcloud_projector
```

### Option 2: Using launch file
```bash
ros2 launch pc2image pointcloud_projector.launch.py
```

## Topics

### Input Topics
- `/camera/camera/color/image_raw` - RGB camera image
- `/camera/camera/color/camera_info` - RGB camera calibration info
- `/camera/camera/depth/image_rect_raw` - Depth camera image
- `/camera/camera/depth/camera_info` - Depth camera calibration info
- `/livox/lidar` - LiDAR point cloud (PointCloud2)

### Output Topics
- `/pc2/proj` - Projected image with LiDAR points overlaid
- `/pc2/diag` - Diagnostic image showing only projected points

## Required Transforms

The node requires the following TF transforms to be published:
- `camera_color_optical_frame` - RGB camera optical frame
- `camera_depth_optical_frame` - Depth camera optical frame
- `livox_frame` - LiDAR frame

## Troubleshooting

### Missing Dependencies
If you encounter import errors, ensure all Python dependencies are installed:
```bash
pip3 install scipy opencv-python numpy
```

### Transform Issues
If you see warnings about missing transforms, ensure your TF tree is properly configured. You can check with:
```bash
ros2 run tf2_tools view_frames
```

### Topic Remapping
If your topics have different names, you can remap them when running:
```bash
ros2 run pc2image pointcloud_projector \
  --ros-args \
  -r /camera/camera/color/image_raw:=/your/color/topic \
  -r /livox/lidar:=/your/lidar/topic
```

Or modify the launch file to include remappings.

## Notes

- The node uses approximate time synchronization with a 100ms tolerance
- Depth colorization uses HSV colormap (red=close, blue=far)
- Points behind the camera plane are automatically filtered out