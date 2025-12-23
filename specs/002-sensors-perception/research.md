# Research: Module 2 - Sensors and Perception for Humanoid Robots

**Date**: 2025-12-07
**Feature**: 002-sensors-perception
**Purpose**: Resolve technical questions and design decisions for sensor perception content

## Overview

This research document consolidates findings on camera systems, depth sensing, IMU/proprioception, and sensor fusion for humanoid robots. All findings inform the educational content design for Module 2, ensuring technical accuracy and appropriate depth for CS students with Python + ROS2 Module 1 knowledge.

---

## 1. Camera Systems for Humanoid Robots

### Research Questions Addressed

**Q1**: What are the practical differences between monocular, stereo, and RGB-D cameras for humanoid robot tasks?

**Findings**:

| Camera Type | Depth Information | Typical Range | Use Cases (Humanoids) | Limitations |
|-------------|-------------------|---------------|------------------------|-------------|
| **Monocular** | None (2D only) | N/A (image-based) | Object detection, visual servoing, face recognition | No depth perception, requires motion/priors for 3D |
| **Stereo** | Triangulation (two cameras) | 0.5-10m | Navigation, obstacle avoidance, 3D reconstruction | Requires calibration, struggles with textureless surfaces |
| **RGB-D** | Active IR projection or ToF | 0.3-4m (structured light), 0.5-10m (ToF) | Grasping, manipulation, indoor navigation | Limited outdoor use (IR interference), narrow FOV |

**Design Decision**: Lesson 1 will present these 3 camera types with trade-off analysis focused on humanoid use cases (manipulation needs depth, outdoor navigation suits stereo, indoor object recognition works with monocular).

---

**Q2**: What ROS2 message types represent camera data?

**Findings**:

1. **sensor_msgs/Image**
   - Fields: `header`, `height`, `width`, `encoding` (e.g., "rgb8", "bgr8", "mono8"), `is_bigendian`, `step`, `data`
   - Purpose: Raw image data from cameras
   - Example encoding: "rgb8" (8-bit RGB), "bgr8" (OpenCV default), "mono16" (depth images)

2. **sensor_msgs/CameraInfo**
   - Fields: `header`, `height`, `width`, `distortion_model`, `D` (distortion coefficients), `K` (intrinsic matrix), `R` (rectification matrix), `P` (projection matrix)
   - Purpose: Camera calibration parameters for 3D reconstruction
   - Paired with Image messages (same timestamp)

3. **sensor_msgs/CompressedImage**
   - Fields: `header`, `format` (e.g., "jpeg", "png"), `data`
   - Purpose: Bandwidth-efficient image transmission

**Design Decision**: Lesson 1 code examples will show Image and CameraInfo message structure. Students will learn to subscribe to `/camera/image_raw` and `/camera/camera_info` topics.

---

**Q3**: What camera placement considerations are specific to humanoid robots?

**Findings** (from Boston Dynamics Atlas, Agility Digit, Tesla Optimus case studies):

- **Head-mounted cameras**: Wide FOV for environment awareness, navigation, human interaction (eye contact simulation)
- **Hand/wrist cameras**: Close-range manipulation, grasping verification, fine motor tasks
- **Chest/torso cameras**: Stable viewpoint (less motion than head), useful for SLAM (Simultaneous Localization and Mapping)
- **Multiple camera configurations**: Panoramic vision (360° awareness), redundancy for occlusion handling

**Design Decision**: Lesson 1 will include a case study showing Tesla Optimus multi-camera setup (head + wrist cameras) with rationale for each placement.

---

**Q4**: What are common camera parameters for humanoid robot vision?

**Findings**:

- **Resolution**: 640×480 (low-cost), 1280×720 (HD standard), 1920×1080 (high-quality object detection)
- **Frame Rate**: 30 FPS (standard), 60 FPS (fast motion tracking), 10-15 FPS (power-constrained)
- **Field of View**: 60-90° (typical RGB cameras), 120-180° (wide-angle for navigation)
- **Depth of Field**: Trade-off between near-field (manipulation, 0.2-1m) and far-field (navigation, 1-10m) focus

**Design Decision**: Lesson 1 will explain these parameters with humanoid-specific trade-offs (e.g., manipulation cameras need high resolution + narrow FOV, navigation cameras need wide FOV + lower resolution acceptable).

---

## 2. Depth Sensing Technologies

### Research Questions Addressed

**Q5**: How do LiDAR sensors work (2D vs 3D scanning)?

**Findings**:

**2D LiDAR** (planar scanning):
- Mechanism: Rotating laser measures distance in a single plane (e.g., 360° horizontal sweep)
- Output: Array of (angle, distance) pairs → sensor_msgs/LaserScan
- Range: 10-30m (outdoor), 0.1-10m (indoor)
- Scan rate: 5-40 Hz
- Use case: Mobile robot obstacle avoidance, floor-level navigation

**3D LiDAR** (volumetric scanning):
- Mechanism: Multiple laser beams at different vertical angles (e.g., 16, 32, 64 channels) rotating horizontally
- Output: Point cloud (x, y, z coordinates) → sensor_msgs/PointCloud2
- Range: 50-100m (automotive-grade), 10-30m (robotics-grade)
- Point density: 300,000-2,000,000 points/second
- Use case: 3D mapping, terrain modeling, outdoor navigation

**Design Decision**: Lesson 2 will contrast 2D vs 3D LiDAR with visual diagrams showing scanning patterns. Emphasize that humanoid robots often use 2D LiDAR (cost, power) but high-end systems (Boston Dynamics Spot) use 3D.

---

**Q6**: What are the differences between LiDAR, structured light, and time-of-flight depth cameras?

**Findings**:

| Technology | Mechanism | Range | Accuracy | Indoor/Outdoor | Cost | Example Sensors |
|------------|-----------|-------|----------|----------------|------|-----------------|
| **LiDAR** | Laser time-of-flight (rotating) | 10-100m | ±3cm | Both (excellent) | $$$ | Velodyne, Ouster |
| **Structured Light** | IR pattern projection + triangulation | 0.5-4m | ±1-2cm | Indoor only (IR interference outdoors) | $ | Kinect v1, Intel RealSense D400 |
| **Time-of-Flight (ToF)** | IR pulse round-trip time (per-pixel) | 0.5-10m | ±1cm | Indoor (limited outdoor) | $$ | Kinect v2 (Azure), PMD sensors |

**Design Decision**: Lesson 2 will present this comparison table with trade-off analysis. Humanoid manipulation scenarios → structured light (precise, short-range). Humanoid navigation → LiDAR (long-range) or ToF (mid-range).

---

**Q7**: What ROS2 message types represent depth data?

**Findings**:

1. **sensor_msgs/LaserScan** (2D LiDAR)
   - Fields: `header`, `angle_min`, `angle_max`, `angle_increment`, `range_min`, `range_max`, `ranges[]`, `intensities[]`
   - Example: 360° scan with 1° resolution → 360 range measurements

2. **sensor_msgs/PointCloud2** (3D LiDAR, depth cameras)
   - Fields: `header`, `height`, `width`, `fields[]` (x, y, z, rgb, intensity), `is_dense`, `data`
   - Binary format for efficiency (millions of points)
   - Tools: `pcl_ros` (Point Cloud Library integration), RViz visualization

3. **sensor_msgs/Range** (single ultrasonic/IR sensor)
   - Fields: `header`, `radiation_type`, `field_of_view`, `min_range`, `max_range`, `range`
   - Use case: Simple proximity sensing (less common in humanoids)

**Design Decision**: Lesson 2 code examples will show LaserScan subscription and PointCloud2 structure. Students will visualize point clouds in RViz.

---

**Q8**: What are the trade-offs for each depth sensing technology?

**Findings**:

**LiDAR**:
- ✅ Pros: Long range, outdoor capability, high accuracy, 360° coverage
- ❌ Cons: Expensive, power-hungry, mechanical parts (durability), struggles with transparent/reflective surfaces

**Structured Light**:
- ✅ Pros: Inexpensive, dense depth maps, good for close-range manipulation
- ❌ Cons: Indoor only, limited range, IR interference in sunlight, fails with transparent objects

**Time-of-Flight**:
- ✅ Pros: Mid-range versatility, simple per-pixel depth, lower cost than LiDAR
- ❌ Cons: Lower accuracy than LiDAR, multipath interference (reflective surfaces), moderate power consumption

**Design Decision**: Lesson 2 will include a decision tree: "Which depth sensor for your humanoid robot?" based on environment (indoor/outdoor), range needs (near/far), and budget.

---

## 3. IMU and Proprioception

### Research Questions Addressed

**Q9**: What do accelerometers, gyroscopes, and magnetometers measure?

**Findings**:

| Sensor | Measures | Units | Use in Humanoids | Drift/Noise |
|--------|----------|-------|------------------|-------------|
| **Accelerometer** | Linear acceleration (3 axes) | m/s² | Detect falls, gravity vector (tilt), dynamic motion | Low drift, high noise (vibrations) |
| **Gyroscope** | Angular velocity (3 axes) | rad/s or deg/s | Rotation rate, orientation changes | High drift (unbounded error accumulation) |
| **Magnetometer** | Magnetic field (3 axes) | μT (microtesla) | Absolute heading (compass), yaw correction | Interference from electronics, ferromagnetic materials |

**9-DOF IMU** (9 Degrees of Freedom): Combines all 3 sensors (3-axis accel + 3-axis gyro + 3-axis mag)

**Design Decision**: Lesson 3 will explain each sensor component with humanoid-specific examples (accelerometer detects robot falling, gyroscope tracks head rotation, magnetometer corrects long-term heading drift).

---

**Q10**: How is IMU data represented in ROS2?

**Findings**:

**sensor_msgs/Imu** message structure:
```python
std_msgs/Header header               # Timestamp + frame_id
geometry_msgs/Quaternion orientation # Estimated orientation (if sensor fusion applied)
float64[9] orientation_covariance    # Uncertainty in orientation
geometry_msgs/Vector3 angular_velocity    # Gyroscope data (rad/s)
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration # Accelerometer data (m/s²)
float64[9] linear_acceleration_covariance
```

**Key Notes**:
- `orientation` may be empty if IMU doesn't compute orientation (raw sensors only provide angular_velocity + linear_acceleration)
- Covariance matrices quantify sensor noise/uncertainty (important for sensor fusion)
- Frame convention: ROS REP 103 (x-forward, y-left, z-up in body frame)

**Design Decision**: Lesson 3 code example will show subscribing to `/imu/data` topic and interpreting angular_velocity (detecting rotation) and linear_acceleration (detecting tilt from gravity).

---

**Q11**: What is sensor drift and how does it affect IMU accuracy?

**Findings**:

**Gyroscope Drift**:
- Mechanism: Small constant bias in angular velocity measurement → integrated error grows unbounded over time
- Effect: Orientation estimate becomes increasingly wrong (e.g., robot thinks it's facing north but actually facing east after 60 seconds)
- Mitigation: Sensor fusion with accelerometer (gravity vector) and magnetometer (magnetic north) to correct drift

**Accelerometer Noise**:
- Mechanism: High-frequency vibrations (walking, motor vibrations) introduce noise
- Effect: Noisy linear acceleration readings, difficult to detect small motions
- Mitigation: Low-pass filtering, sensor fusion with gyroscope (smooth short-term, correct long-term)

**Design Decision**: Lesson 3 will include a visual diagram showing gyroscope drift accumulation over time and why sensor fusion is necessary (preview of Lesson 4).

---

**Q12**: How do IMUs enable balance control for bipedal robots?

**Findings** (from humanoid robotics literature):

**Balance Control Loop**:
1. IMU measures body tilt (accelerometer gravity vector) and rotation rate (gyroscope)
2. Controller estimates center of mass (COM) position relative to feet
3. If COM drifts outside support polygon (foot contact area), robot is falling
4. Controller commands corrective actions:
   - **Ankle strategy**: Adjust ankle torque to shift COM back (small disturbances)
   - **Hip strategy**: Swing hips to shift COM (medium disturbances)
   - **Step strategy**: Take a step to reposition feet under COM (large disturbances)

**IMU Role**: Provides real-time feedback (100-1000 Hz) on body orientation → enables rapid balance corrections before fall occurs.

**Design Decision**: Lesson 3 will explain this balance control loop conceptually with a case study from Boston Dynamics Atlas (IMU-based fall detection and recovery).

---

**Q13**: What is proprioception in robotics?

**Findings**:

**Definition**: Robot's sense of its own body state (position, velocity, forces) without external sensing.

**Components for Humanoid Proprioception**:
1. **Joint Encoders**: Measure joint angles (e.g., knee at 45°, elbow at 90°)
2. **Joint Velocity Sensors**: Measure joint angular velocities (or derived from encoder)
3. **Joint Torque/Force Sensors**: Measure forces at joints (strain gauges, current sensing)
4. **IMU**: Measures body orientation and acceleration (trunk/torso IMU)
5. **Foot Contact Sensors**: Pressure sensors, force/torque sensors in feet (detect ground contact)

**Proprioceptive State** = (all joint angles, all joint velocities, body orientation, body angular velocity, contact forces)

**Use Case**: Enable robot to know "I'm standing on my left foot with my right leg bent at 30° and my torso tilted 5° forward" without cameras.

**Design Decision**: Lesson 3 will define proprioception and show how IMU is ONE component of a larger proprioceptive system (distinguish from exteroception = external sensors like cameras).

---

## 4. Sensor Fusion Techniques

### Research Questions Addressed

**Q14**: What is sensor fusion and why is it necessary?

**Findings**:

**Definition**: Combining data from multiple sensors to produce more accurate, reliable, and robust information than any single sensor alone.

**Why Necessary**:
- **Complementary strengths**: Cameras fail in darkness (IMU still works), LiDAR struggles with transparent surfaces (cameras can see glass)
- **Redundancy**: If one sensor fails (camera lens gets dirty), other sensors maintain functionality
- **Noise reduction**: Averaging/weighting multiple sensor readings reduces measurement noise
- **Conflicting information resolution**: When sensors disagree (e.g., camera says "no obstacle" but LiDAR detects wall), fusion resolves conflict

**Humanoid-Specific Motivation**: Humanoid robots operate in complex, unpredictable environments (homes, offices, outdoor spaces) where no single sensor is sufficient.

**Design Decision**: Lesson 4 will open with a motivating scenario: "Humanoid robot navigating a home with both bright sunlight (camera overexposed) and glass doors (LiDAR can't see). Why sensor fusion matters."

---

**Q15**: What are Kalman filters and complementary filters?

**Findings**:

**Kalman Filter** (conceptual understanding):
- Purpose: Optimal state estimation from noisy measurements
- How it works (simplified):
  1. **Prediction step**: Use motion model to predict current state (e.g., "robot should be at position X based on velocity")
  2. **Update step**: Incorporate new sensor measurement, weighted by sensor uncertainty (e.g., "GPS says position Y, but GPS has high uncertainty, so don't trust it fully")
  3. Iterative process → state estimate converges to most likely value
- Use case: Combining IMU (high-rate, drifts) with GPS (low-rate, accurate) for robot localization

**Complementary Filter** (simpler alternative):
- Purpose: Combine high-frequency sensor (gyroscope) with low-frequency sensor (accelerometer) for orientation estimation
- How it works:
  - High-pass filter gyroscope (trust short-term, ignore drift)
  - Low-pass filter accelerometer (trust long-term gravity vector, ignore vibration noise)
  - Weighted sum: `orientation = α * gyro_orientation + (1-α) * accel_orientation`
- Use case: IMU orientation estimation (cheaper than full Kalman filter)

**Design Decision**: Lesson 4 will explain Kalman filter conceptually (NOT mathematical derivation with matrices) and show complementary filter as a simpler fusion strategy students can understand intuitively.

---

**Q16**: What are common sensor fusion strategies for humanoid robots?

**Findings**:

1. **Visual-Inertial Odometry (VIO)**
   - Sensors: Camera + IMU
   - Fusion: Track visual features in camera, use IMU to predict motion between frames
   - Use case: Robot localization indoors (where GPS unavailable)
   - Algorithm: Extended Kalman Filter (EKF) or Factor Graph Optimization

2. **Depth-Enhanced Object Detection**
   - Sensors: RGB camera + depth camera (or LiDAR)
   - Fusion: Detect objects in RGB image, use depth to measure distance and size
   - Use case: Grasping (robot needs to know object is 0.5m away, not just "in image")
   - Algorithm: Early fusion (combine RGB+depth before neural network) or late fusion (detect in RGB, filter by depth)

3. **Multi-IMU Body State Estimation**
   - Sensors: Multiple IMUs on different body segments (torso, thighs, feet)
   - Fusion: Estimate full-body pose and dynamics
   - Use case: Balance control, fall detection
   - Algorithm: Unscented Kalman Filter (UKF) for nonlinear dynamics

4. **LiDAR-Camera Calibration for SLAM**
   - Sensors: LiDAR + Camera
   - Fusion: Align LiDAR 3D points with camera pixels (extrinsic calibration), use camera for texture, LiDAR for geometry
   - Use case: 3D mapping of environment
   - Algorithm: Iterative Closest Point (ICP) for alignment, graph SLAM for map building

**Design Decision**: Lesson 4 will present Visual-Inertial Odometry and Depth-Enhanced Object Detection as primary case studies (most relevant to humanoid manipulation and navigation).

---

**Q17**: How does ROS2 robot_localization package combine multiple sensor sources?

**Findings**:

**robot_localization Package** (ROS2 port of ROS1 package):
- Purpose: Fuse odometry, IMU, GPS, and other sensors to produce smooth, accurate pose estimate
- Algorithms: Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF)
- Input topics:
  - `/odom` (wheel odometry or visual odometry)
  - `/imu/data` (IMU measurements)
  - `/gps/fix` (GPS position)
  - `/vo` (visual odometry)
- Output topics:
  - `/odometry/filtered` (fused pose estimate)
  - `/accel/filtered` (fused acceleration)
- Configuration: YAML file specifying which sensors to trust for position/velocity/orientation

**Example Configuration** (for humanoid robot):
```yaml
ekf_localization_node:
  frequency: 30
  odom0: /visual_odometry/odom  # Trust for x, y, yaw
  imu0: /imu/data               # Trust for roll, pitch, yaw velocity
  odom0_config: [true, true, false, false, false, true, ...]
  imu0_config: [false, false, false, true, true, true, ...]
```

**Design Decision**: Lesson 4 will reference robot_localization as a real-world ROS2 tool for sensor fusion, showing configuration file conceptually (not requiring students to run it, but understanding what it does).

---

**Q18**: What are the challenges of sensor synchronization and temporal alignment?

**Findings**:

**Problem**: Different sensors have different update rates and latencies:
- IMU: 100-1000 Hz, low latency (<1ms)
- Camera: 30-60 Hz, moderate latency (10-50ms due to processing)
- LiDAR: 10-40 Hz, low latency (<5ms)
- GPS: 1-10 Hz, high latency (100-1000ms)

**Challenge**: If sensor measurements aren't time-aligned, fusion produces incorrect results (e.g., fusing 100ms-old GPS with current IMU → robot thinks it's in wrong location).

**Solutions**:
1. **Hardware Synchronization**: Use external trigger to synchronize sensor clocks (expensive, complex)
2. **Software Timestamp Alignment**: Buffer sensor messages, interpolate to common timestamp (ROS2 `message_filters` package)
3. **Latency Compensation**: Predict sensor measurement forward/backward in time based on motion model

**ROS2 Approach**: All sensor messages have `header.stamp` (timestamp). Fusion algorithms (like robot_localization) automatically handle timestamp alignment.

**Design Decision**: Lesson 4 will briefly mention timestamp synchronization as a practical challenge with visual diagram showing misaligned sensor readings. Emphasize that ROS2 message_filters and robot_localization handle this automatically.

---

## 5. ROS2 Visualization and Simulation Tools

### Research Questions Addressed

**Q19**: How can RViz visualize camera images, point clouds, and IMU data?

**Findings**:

**RViz** (ROS2 Visualization tool):

1. **Camera Images**:
   - Display type: `Image` (subscribe to sensor_msgs/Image topic)
   - Visualization: Render image in 2D overlay panel
   - Interaction: Zoom, pan, inspect pixel values

2. **Point Clouds**:
   - Display type: `PointCloud2` (subscribe to sensor_msgs/PointCloud2 topic)
   - Visualization: Render 3D points in space, color by intensity/RGB/height
   - Interaction: Rotate 3D view, filter by distance, adjust point size

3. **IMU Data**:
   - Display type: `Axes` (visualize orientation quaternion)
   - OR `TF` (Transform frames showing IMU coordinate frame rotating)
   - Visualization: 3D axes (X-red, Y-green, Z-blue) showing IMU orientation
   - Interaction: Watch axes rotate as robot moves

**Multi-Sensor Visualization**:
- RViz can display multiple sensor types simultaneously (e.g., camera image in overlay + point cloud in 3D + IMU axes)
- Configuration saved in `.rviz` file for reuse

**Design Decision**: Each lesson (1-4) will include an "RViz Visualization" section showing how to visualize the sensor type covered. Lesson 4 (sensor fusion) will show multi-sensor RViz configuration.

---

**Q20**: What simulation tools can generate synthetic sensor data?

**Findings**:

1. **Gazebo Classic** (ROS2 compatible):
   - Camera plugins: Generate sensor_msgs/Image from simulated cameras
   - LiDAR plugins: Generate sensor_msgs/LaserScan and PointCloud2 from ray-casting
   - IMU plugin: Generate sensor_msgs/Imu from simulated rigid body dynamics
   - Use case: Test perception algorithms without real hardware

2. **NVIDIA Isaac Sim** (advanced):
   - High-fidelity camera simulation (realistic lighting, reflections)
   - LiDAR simulation with material properties (reflectivity, absorption)
   - IMU simulation with configurable noise models
   - ROS2 bridge for publishing simulated sensor data
   - Use case: Photorealistic training data for ML models

3. **Webots** (alternative):
   - Similar capabilities to Gazebo (camera, LiDAR, IMU simulation)
   - ROS2 interface via webots_ros2 package
   - Use case: Educational robotics (simpler than Gazebo, good for beginners)

**Design Decision**: Module 2 content will reference Gazebo for sensor simulation (students already familiar from potential Module 1 exercises). NVIDIA Isaac Sim mentioned as advanced tool (preview of Module 3).

---

**Q21**: What are best practices for visualizing multi-sensor data simultaneously?

**Findings**:

**RViz Best Practices**:
1. **Fixed Frame**: Set to robot base frame (`base_link`) so sensors move relative to robot
2. **Color Coding**: Use distinct colors for different sensor types (camera = default, LiDAR = green points, IMU axes = RGB)
3. **Transparency**: Make point clouds semi-transparent so overlapping data visible
4. **Separate Panels**: Camera images in 2D panels, 3D sensors (LiDAR, IMU) in 3D viewport
5. **Selective Display**: Toggle sensors on/off to reduce visual clutter

**Workflow**:
1. Start with one sensor type, verify data looks correct
2. Add second sensor, check alignment (TF frames match)
3. Add remaining sensors, adjust visualization properties (size, color, transparency)
4. Save RViz config file (`.rviz`) for reuse

**Design Decision**: Lesson 4 capstone project will guide students through creating a multi-sensor RViz configuration (camera + LiDAR + IMU) and saving it as `multi_sensor.rviz`.

---

## Summary of Research Findings

### Key Decisions for Content Design

1. **Lesson 1 (Camera Systems)**:
   - Cover monocular, stereo, RGB-D with trade-off table
   - Explain sensor_msgs/Image and CameraInfo message structures
   - Case study: Tesla Optimus multi-camera placement
   - RViz: Visualize camera images from simulated robot

2. **Lesson 2 (Depth Sensing)**:
   - Compare LiDAR, structured light, ToF with range/accuracy table
   - Explain sensor_msgs/LaserScan and PointCloud2 message structures
   - Decision tree: Which depth sensor for your robot?
   - RViz: Visualize point clouds from LiDAR simulation

3. **Lesson 3 (IMU & Proprioception)**:
   - Explain accelerometer, gyroscope, magnetometer principles
   - Explain sensor_msgs/Imu message structure
   - Diagram: Gyroscope drift over time (why fusion needed)
   - Case study: Boston Dynamics Atlas balance control
   - Define proprioception (IMU + joint encoders + contact sensors)
   - RViz: Visualize IMU orientation as 3D axes

4. **Lesson 4 (Sensor Fusion)**:
   - Motivating scenario: Multi-sensor redundancy in complex environment
   - Explain Kalman filter (conceptual) and complementary filter (intuitive)
   - Case studies: Visual-Inertial Odometry, Depth-Enhanced Object Detection
   - Reference ROS2 robot_localization package
   - Timestamp synchronization challenges
   - RViz: Multi-sensor configuration (camera + LiDAR + IMU)

5. **Capstone Project**:
   - Design a multi-sensor perception system for a humanoid robot task (e.g., "Navigate indoor office and pick up objects from desk")
   - Specify sensors (camera type, depth sensor, IMU placement)
   - Justify fusion strategy
   - Sketch ROS2 node architecture

6. **Quiz**:
   - Conceptual questions on sensor types, ROS2 messages, fusion strategies
   - Code-reading questions: Interpret sensor_msgs/Image, PointCloud2, Imu
   - Scenario-based questions: Choose appropriate sensors for given task

### Technical Accuracy Sources

- ROS2 official documentation: https://docs.ros.org/en/humble/
- sensor_msgs package: https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- Boston Dynamics Atlas technical papers (balance control, IMU-based fall detection)
- Agility Robotics Digit documentation (camera + LiDAR integration)
- Tesla Optimus presentations (multi-camera system design)
- IEEE Robotics & Automation Letters (sensor fusion algorithms)

### Unresolved Questions

None. All research questions from plan.md resolved. Ready to proceed to Phase 1 (data-model.md, contracts, quickstart).

---

**Status**: ✅ Research complete. Findings inform Lesson 1-4 content design, code examples, case studies, and RViz visualization guidance.
