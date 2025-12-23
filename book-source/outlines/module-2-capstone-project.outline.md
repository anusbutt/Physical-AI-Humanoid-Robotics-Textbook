# Outline: Module 2 Capstone Project - Integrated Sensor System Design

**Module**: Module 2 - Sensors and Perception for Humanoid Robots
**Content File**: 05-capstone-project.md
**Target Audience**: CS students with Python knowledge who completed all 4 Module 2 lessons
**Outline Created**: 2025-12-13
**Status**: Ready for Content Writer Agent

---

## Overview

**Purpose**: Synthesize all Module 2 concepts (camera systems, depth sensing, IMU/proprioception, sensor fusion) through a comprehensive design challenge that mirrors real-world humanoid robotics system engineering.

**Pedagogical Goals**:
- Apply camera, depth, IMU, and fusion concepts in integrated scenarios
- Justify sensor selection with technical reasoning and trade-off analysis
- Design ROS2 architectures for multi-sensor perception systems
- Analyze failure modes and degradation strategies
- Communicate technical designs through documentation

**Deliverable**: 2-3 page design document (PDF or Markdown) covering sensor selection, ROS2 architecture, fusion strategy, and failure analysis for one scenario.

**Estimated Total Length**: 1,800-2,200 words

---

## Section 1: Introduction

**Purpose**: Motivate the capstone project and establish learning objectives

**Planned Content**:

### Opening Hook (1 paragraph, 50-75 words)
- Real-world context: Professional roboticists design sensor systems iteratively, balancing performance, cost, reliability, and computational constraints
- Challenge: Students will experience this process by designing a complete perception system for a humanoid robot
- Connection to all 4 lessons: This integrates camera systems (L1), depth sensing (L2), IMU/proprioception (L3), and sensor fusion (L4)

### Capstone Purpose (2-3 paragraphs, 150-200 words)
- **Why a design document vs. code implementation?**
  - Professional robotics begins with architecture and justification before coding
  - Design documents force explicit reasoning about trade-offs (can't hide behind "I'll try different sensors")
  - Mirrors industry practice (RFP responses, technical design reviews, architecture documentation)

- **Learning through integration**:
  - Individual lessons covered sensors in isolation; real humanoids require coordinated multi-sensor systems
  - Trade-offs become explicit: camera resolution vs. compute budget, LiDAR range vs. cost, IMU drift vs. fusion complexity
  - Failure analysis reveals sensor limitations learned in L1-L4

- **Preparing for professional practice**:
  - Design documentation skills are critical for robotics engineers (grant proposals, technical reviews, team collaboration)
  - Justification with evidence (not arbitrary choices) distinguishes professional work
  - This capstone mimics early-stage system design in real robotics companies

### Learning Objectives (bulleted list, 75-100 words)
Students will be able to:
- **Apply** camera, depth sensor, IMU, and fusion concepts to solve realistic humanoid robotics challenges
- **Justify** sensor selection with technical trade-offs (resolution vs. latency, range vs. accuracy, cost vs. redundancy)
- **Design** ROS2 node architectures for multi-sensor perception with appropriate message types and data flow
- **Analyze** failure modes and propose graceful degradation strategies
- **Communicate** technical decisions through structured design documentation

**Estimated Length**: 250-300 words

---

## Section 2: Capstone Scenario Options

**Purpose**: Provide 4 distinct scenarios requiring all Module 2 concepts

**Structure**: Each scenario presented as:
1. Scenario description and primary challenge
2. Required sensor capabilities (linking to L1-L4)
3. Key technical challenges
4. Success criteria

---

### Scenario A: Indoor Home Assistant Robot

**Scenario Description** (100-125 words):

You are designing a humanoid robot to assist with household tasks in a typical urban apartment. The robot must:
- **Navigate autonomously** through rooms with varying lighting (bright kitchen, dim bedroom), avoiding furniture, pets, and humans
- **Identify and manipulate objects** such as picking up a book from a coffee table, retrieving a mug from a shelf at eye level, or placing items in drawers
- **Maintain balance** while walking on different surfaces (tile, carpet, rugs) and carrying loads up to 5 kg
- **Operate indoors only** (no outdoor navigation, weather resistance not required)

**Key constraints**: Limited compute budget (embedded GPU or CPU-only processing), must work in apartments with glass surfaces and mirrors, cost target under $5,000 for all sensors.

**Required Sensor Capabilities** (linking to lessons):

1. **Camera Systems (Lesson 1)**:
   - **Object recognition**: Distinguish between common household items (cups, books, furniture, pets)
   - **Visual servoing**: Track objects during manipulation for precise grasping
   - **Localization**: Identify visual landmarks (doorways, furniture) for navigation
   - *Challenge*: Lighting variability (sunlight through windows vs. evening lamp lighting)

2. **Depth Sensing (Lesson 2)**:
   - **Obstacle avoidance**: Detect furniture, walls, humans at 0.5-5m range for safe navigation
   - **Grasp distance measurement**: Determine 3D position of objects for reach planning (0.3-2m)
   - **Surface detection**: Identify floor level, table surfaces, shelf heights
   - *Challenge*: Glass and mirrors (IR-based depth sensors may fail)

3. **IMU and Proprioception (Lesson 3)**:
   - **Balance control**: Detect tilt/lean while walking or carrying objects
   - **Fall detection**: Emergency stop if robot begins to tip beyond recovery threshold
   - **Motion prediction**: Estimate velocity and acceleration for stable walking
   - *Challenge*: Drift accumulation during long tasks (10+ minutes of operation)

4. **Sensor Fusion (Lesson 4)**:
   - **Visual-Inertial Odometry (VIO)**: Combine camera + IMU for GPS-free localization indoors
   - **Obstacle + balance fusion**: Merge depth sensor obstacle data with IMU tilt to adjust gait (slow down when tilting)
   - **Redundancy**: If camera fails (bright sunlight glare), rely on depth + IMU for basic navigation
   - *Challenge*: Synchronizing sensors with different update rates (camera 15 Hz, IMU 100 Hz, depth 30 Hz)

**Key Technical Challenges** (75-100 words):
- **Lighting variation**: Morning sunlight vs. evening dim lighting affects camera and RGB-D sensors differently
- **Reflective surfaces**: Glass tables, mirrors confuse depth sensors (structured light, ToF)
- **Compute constraints**: High-resolution cameras + dense depth processing may exceed budget
- **Precision requirements**: Grasping requires <2 cm position error; navigation tolerates 10 cm error
- **Cost vs. performance**: Balancing sensor quality with $5,000 budget

**Success Criteria** (50-75 words):
- Sensor suite enables object detection, distance measurement, balance control, and localization
- Fusion strategy uses VIO or similar for drift-free indoor navigation
- Failure modes addressed: camera glare, depth sensor glass failures, IMU drift
- ROS2 architecture shows proper topic structure (`sensor_msgs/Image`, `PointCloud2`, `Imu`)
- Justifications reference specific trade-offs from Lessons 1-4

**Estimated Length**: 350-400 words

---

### Scenario B: Outdoor Delivery Robot

**Scenario Description** (100-125 words):

You are designing a humanoid robot for outdoor package delivery on university campuses and office parks. The robot must:
- **Walk on uneven terrain** including grass, gravel paths, curbs, and occasional stairs
- **Avoid dynamic obstacles** such as pedestrians, bicyclists, vehicles, and moving doors
- **Navigate long distances** (up to 2 km per trip) using GPS-aided localization
- **Operate in variable weather** (sunny, cloudy, light rain) during daylight hours
- **Carry packages** up to 15 kg while maintaining stability

**Key constraints**: Outdoor lighting (direct sunlight, shadows), long-range obstacle detection (5-30m), battery-limited compute power, $10,000 sensor budget.

**Required Sensor Capabilities** (linking to lessons):

1. **Camera Systems (Lesson 1)**:
   - **Semantic navigation**: Identify sidewalks, crosswalks, building entrances
   - **Visual landmarks**: Recognize buildings, signs for route-following
   - **Texture analysis**: Distinguish pavement from grass, detect curbs
   - *Challenge*: Direct sunlight causes overexposure; shadows create high contrast

2. **Depth Sensing (Lesson 2)**:
   - **Long-range obstacle detection**: Detect pedestrians, vehicles at 5-30m for path planning
   - **Terrain mapping**: Build 3D map of ground surface for footstep planning on uneven terrain
   - **Curb detection**: Identify height changes (stairs, curbs) for gait adaptation
   - *Challenge*: Sunlight interferes with IR-based sensors; need LiDAR for outdoor reliability

3. **IMU and Proprioception (Lesson 3)**:
   - **Tilt/balance on slopes**: Detect incline changes (grass slopes, ramps) for posture adjustment
   - **Vibration damping**: Filter out ground vibration from gravel/uneven surfaces
   - **Package load sensing**: Detect center-of-mass shift when carrying 15 kg package
   - *Challenge*: Magnetic interference near buildings (affects magnetometer calibration)

4. **Sensor Fusion (Lesson 4)**:
   - **LiDAR + IMU SLAM**: Fuse 3D LiDAR scans with IMU for robust outdoor localization
   - **GPS + Visual Odometry**: Combine GPS (10m accuracy) with visual odometry (cm-level relative accuracy)
   - **Multi-modal obstacle fusion**: Merge LiDAR point clouds with stereo camera depth for comprehensive obstacle map
   - *Challenge*: GPS dropout in tree cover or near buildings requires seamless fallback to VIO

**Key Technical Challenges** (75-100 words):
- **Sunlight interference**: RGB-D sensors fail outdoors; must use LiDAR or stereo cameras
- **Long-range detection**: Need 30m range for safe navigation at walking speed (1-2 m/s)
- **Compute vs. battery**: Dense LiDAR processing (100k points/sec) drains battery quickly
- **Weather robustness**: Light rain may affect camera visibility and LiDAR returns
- **Terrain adaptation**: IMU must detect slope changes quickly (<0.5 sec) for gait adjustment

**Success Criteria** (50-75 words):
- Sensor suite handles outdoor lighting, long-range obstacles, uneven terrain, and package loads
- LiDAR selected over RGB-D for outdoor reliability (justified with sunlight limitations from L2)
- Fusion strategy uses LiDAR+IMU SLAM with GPS fallback
- Failure modes addressed: GPS dropout, camera sun glare, IMU magnetic interference
- ROS2 architecture includes `sensor_msgs/LaserScan` or `PointCloud2`, `NavSatFix`, `Imu`

**Estimated Length**: 350-400 words

---

### Scenario C: Human Interaction Robot (Social/Healthcare)

**Scenario Description** (100-125 words):

You are designing a humanoid robot for safe, natural interaction with humans in healthcare settings (hospitals, nursing homes). The robot must:
- **Approach humans safely** without startling or colliding, maintaining socially appropriate distance (0.5-1.5m)
- **Track faces and gestures** to understand non-verbal communication and maintain eye contact
- **Hand off objects** such as medication containers, water bottles, or personal items with safe transfer
- **Maintain stable head orientation** for natural gaze direction during conversations
- **Detect human distress** (fallen person, sudden movements) and alert caregivers

**Key constraints**: Human safety is paramount (no collisions), privacy considerations (minimal data storage), indoor operation, $7,500 sensor budget.

**Required Sensor Capabilities** (linking to lessons):

1. **Camera Systems (Lesson 1)**:
   - **Face detection and tracking**: Identify human faces for gaze direction and attention
   - **Gesture recognition**: Detect waving, pointing, reaching gestures for interaction cues
   - **Object handoff visual servoing**: Track hand position during object transfer
   - *Challenge*: Privacy concerns (avoid recording/storing facial images beyond immediate processing)

2. **Depth Sensing (Lesson 2)**:
   - **Safe approach distance**: Measure distance to humans (0.5-3m) for personal space boundaries
   - **3D person tracking**: Combine RGB face detection with depth for full 3D position estimation
   - **Obstacle detection**: Avoid medical equipment (IV poles, wheelchairs) in cluttered environments
   - *Challenge*: Wheelchair users require lower sensor placement; standard chest-mounted depth may miss obstacles

3. **IMU and Proprioception (Lesson 3)**:
   - **Stable head orientation**: Keep head level during walking for natural gaze (no bobbing)
   - **Gentle motion control**: Smooth acceleration/deceleration to avoid startling humans
   - **Collision reaction**: Detect unexpected contact (bumping into human) via accelerometer spike
   - *Challenge*: Distinguish intentional touch (human patting robot) from collision

4. **Sensor Fusion (Lesson 4)**:
   - **Camera + Depth 3D person tracking**: Fuse RGB face detection with depth for robust human localization
   - **IMU + Vision head stabilization**: Use IMU to predict head motion, compensate camera tracking for smooth gaze
   - **Multi-sensor safety layer**: Redundant human detection (camera + depth) to prevent collision if one fails
   - *Challenge*: Synchronizing camera face detection (15 Hz) with depth sensor (30 Hz) for smooth tracking

**Key Technical Challenges** (75-100 words):
- **Privacy and data security**: Minimize facial image storage, use edge processing only
- **Human safety**: Redundant human detection to ensure zero collision risk
- **Natural interaction**: Head stabilization for human-like gaze behavior
- **Variable human heights**: Children, seated adults, wheelchair users require adjustable sensor FOV
- **Close-range precision**: Object handoff requires <5 cm position accuracy at 0.5-1m range

**Success Criteria** (50-75 words):
- Sensor suite enables face tracking, safe distance measurement, stable head orientation, and gesture recognition
- Fusion strategy uses camera+depth for 3D person tracking with IMU head stabilization
- Privacy considerations addressed (edge processing, no persistent image storage)
- Failure modes: camera occlusion (back turned), depth sensor close-range limits, redundant human detection
- ROS2 architecture shows message_filters for camera+depth synchronization

**Estimated Length**: 350-400 words

---

### Scenario D: Warehouse Logistics Robot

**Scenario Description** (100-125 words):

You are designing a humanoid robot for high-speed warehouse logistics (e.g., Amazon fulfillment centers). The robot must:
- **Navigate warehouse aisles** at high speed (up to 3 m/s) avoiding forklifts, human workers, and other robots
- **Identify shelf locations** using barcode or QR code scanning for item retrieval
- **Pick items from shelves** at varying heights (floor level to 2.5m overhead)
- **Track rapid acceleration** during start/stop cycles to maintain balance
- **Operate in structured environment** with known floor plan but dynamic obstacles

**Key constraints**: High-speed navigation requires low-latency sensors, precise localization (<5 cm error), harsh lighting (overhead fluorescents), $12,000 sensor budget.

**Required Sensor Capabilities** (linking to lessons):

1. **Camera Systems (Lesson 1)**:
   - **Barcode/QR code scanning**: High-resolution camera (1080p+) for item identification at 0.5-2m
   - **Multi-camera visual odometry**: Stereo or multi-camera setup for precise localization in GPS-free warehouse
   - **Shelf edge detection**: Identify shelf boundaries for precise reaching
   - *Challenge*: Overhead fluorescent lighting causes flickering at 60 Hz (affects camera exposure)

2. **Depth Sensing (Lesson 2)**:
   - **High-speed obstacle detection**: LiDAR with low latency (<50 ms) for safe navigation at 3 m/s
   - **Precise shelf localization**: 3D mapping for shelf slot identification (±2 cm accuracy)
   - **Item dimension measurement**: Depth camera for estimating item size before grasping
   - *Challenge*: Fast motion (3 m/s) may cause motion blur in depth cameras; LiDAR preferred

3. **IMU and Proprioception (Lesson 3)**:
   - **Rapid acceleration tracking**: 200+ Hz IMU update rate for detecting sudden starts/stops
   - **Balance during high-speed turns**: Detect lateral acceleration for dynamic stability control
   - **Vibration isolation**: Filter out floor vibration from concrete warehouse floors
   - *Challenge*: High update rate IMU generates significant data throughput (bandwidth constraint)

4. **Sensor Fusion (Lesson 4)**:
   - **Multi-camera visual odometry**: Fuse 3+ cameras for redundant, drift-free localization
   - **LiDAR + Visual SLAM**: Combine LiDAR structure with visual features for robust warehouse mapping
   - **IMU-aided camera stabilization**: Use IMU prediction to compensate for camera motion blur during acceleration
   - *Challenge*: Fusing high-rate IMU (200 Hz) with lower-rate cameras (30 Hz) requires predictive filtering

**Key Technical Challenges** (75-100 words):
- **Low-latency requirements**: 3 m/s speed requires obstacle detection within 50 ms to stop in time
- **Precise localization**: Shelf slot accuracy (<5 cm) demands high-quality SLAM and calibration
- **Harsh lighting**: Fluorescent flicker may interfere with camera exposure timing
- **High data throughput**: Multiple cameras + LiDAR + high-rate IMU exceeds typical ROS2 bandwidth
- **Dynamic obstacles**: Humans, forklifts, other robots require real-time path replanning

**Success Criteria** (50-75 words):
- Sensor suite enables high-speed navigation, barcode scanning, precise shelf localization, rapid acceleration tracking
- Fusion strategy uses multi-camera visual odometry with LiDAR and IMU integration
- Latency analysis shows <50 ms obstacle detection pipeline
- Failure modes: camera motion blur, LiDAR occlusion by forklifts, IMU saturation during emergency stops
- ROS2 architecture addresses data throughput (QoS settings, compression, prioritization)

**Estimated Length**: 350-400 words

---

## Section 3: Design Requirements

**Purpose**: Specify what students must include in their design document

**Structure**: Each requirement explained with examples and guidance

**Planned Content**:

### 3.1 Sensor Selection and Justification (100-125 words)

Students must specify for their chosen scenario:

**Camera System**:
- **Type**: Monocular RGB, stereo, RGB-D, or combination?
- **Specifications**: Resolution (e.g., 640×480, 1920×1080), frame rate (15-60 FPS), field of view (60-120°)
- **Example models**: Intel RealSense D435 (RGB-D), ZED 2 (stereo), generic USB webcam (monocular)
- **Justification**: Why this type and spec? Reference trade-offs from Lesson 1 (e.g., "1080p chosen for barcode scanning accuracy despite higher compute cost")

**Depth Sensor**:
- **Type**: LiDAR (mechanical/solid-state), structured light RGB-D, ToF depth camera?
- **Specifications**: Range (0.5-30m), accuracy (±2 cm to ±10 cm), update rate (10-100 Hz)
- **Example models**: Velodyne Puck (LiDAR), Intel RealSense L515 (LiDAR), Azure Kinect (ToF)
- **Justification**: Why this technology? Reference Lesson 2 limitations (e.g., "LiDAR selected over RGB-D for outdoor operation due to sunlight interference")

**IMU**:
- **Specifications**: Update rate (50-200 Hz), accelerometer range (±2g to ±16g), gyroscope range (±250°/s to ±2000°/s)
- **Example models**: MPU-9250 (low-cost 9-axis), VectorNav VN-100 (high-precision), Xsens MTi (industrial)
- **Justification**: Why this update rate and range? Reference Lesson 3 trade-offs (e.g., "200 Hz IMU for warehouse scenario to track rapid acceleration during emergency stops")

### 3.2 Sensor Placement and Coverage (100-125 words)

Students must diagram sensor placement on humanoid robot:

**Placement Considerations**:
- **Head-mounted**: Camera for navigation, face tracking, visual landmarks (can pan/tilt for active viewing)
- **Chest-mounted**: LiDAR or depth camera for stable SLAM reference frame (minimal occlusion by arms)
- **Wrist-mounted**: Camera for manipulation visual servoing (eye-in-hand configuration)
- **Torso IMU**: Central location for whole-body balance sensing

**Field-of-View Coverage**:
- Diagram FOV cones showing what each sensor "sees"
- Identify coverage gaps (e.g., "blind spot behind robot, mitigated by rotation before backing up")
- Redundancy strategy (e.g., "overlapping camera + LiDAR FOV in front arc for obstacle detection redundancy")

**Example**: Simple labeled robot sketch showing sensor positions and FOV angles

### 3.3 ROS2 Integration Architecture (125-150 words)

Students must specify ROS2 node structure:

**Node Architecture**:
- **Sensor driver nodes**: `camera_driver_node`, `lidar_driver_node`, `imu_driver_node` (published topics)
- **Processing nodes**: `object_detector_node`, `slam_node`, `balance_controller_node` (subscriptions + publications)
- **Fusion node**: `sensor_fusion_node` (subscribes to multiple sensors, publishes fused state estimate)

**Topic Names and Message Types** (must reference actual ROS2 message types):
- Camera: `/camera/image_raw` (sensor_msgs/Image), `/camera/camera_info` (sensor_msgs/CameraInfo)
- Depth: `/depth/points` (sensor_msgs/PointCloud2) or `/scan` (sensor_msgs/LaserScan)
- IMU: `/imu/data` (sensor_msgs/Imu)
- Fused output: `/odometry/filtered` (nav_msgs/Odometry) from robot_localization package

**QoS Policies**:
- Best effort vs. reliable: cameras typically best effort (real-time priority over guaranteed delivery)
- Queue depth: 5-10 for high-rate sensors (cameras, IMU), 1-2 for slower control commands

**Diagram Requirement**: Hand-drawn or tool-generated (draw.io, rqt_graph) showing nodes, topics, message types

### 3.4 Sensor Fusion Strategy (125-150 words)

Students must specify fusion algorithm and data flow:

**Fusion Algorithm Selection**:
- **Complementary Filter** (Lesson 4): Suitable for IMU + low-rate orientation sensor (e.g., magnetometer), simple and low-compute
- **Extended Kalman Filter (EKF)**: For camera + IMU (VIO), handles nonlinear motion models
- **robot_localization package**: Pre-built ROS2 EKF for fusing IMU, wheel odometry, visual odometry, GPS
- **Visual-Inertial Odometry (VIO)**: Camera + IMU for drift-free localization (ORB-SLAM, VINS-Mono)
- **LiDAR + IMU SLAM**: 3D LiDAR scans + IMU for outdoor mapping (LOAM, LIO-SAM)

**State Variables to Estimate**:
- Position (x, y, z in global frame)
- Orientation (roll, pitch, yaw)
- Linear velocity (vx, vy, vz)
- Angular velocity (ωx, ωy, ωz)

**Which Sensors Contribute to Which States?**:
- Example: "Camera provides position via visual odometry; IMU provides orientation and angular velocity; depth sensor provides obstacle constraints to prevent position drift into walls"

**Fusion Block Diagram**: Data flow from sensors → fusion algorithm → state estimate output

### 3.5 Failure Modes and Graceful Degradation (125-150 words)

Students must analyze 3-5 failure scenarios:

**Failure Mode Examples**:

1. **Camera Failure** (e.g., lens obscured by dirt, bright sun glare):
   - **Detection**: No new image messages for >1 second, or all pixels saturated (overexposure)
   - **Degradation Strategy**: Fall back to depth sensor + IMU for basic obstacle avoidance and dead reckoning; disable object recognition tasks
   - **Performance Impact**: Cannot identify objects or visual landmarks, navigation limited to known areas

2. **Depth Sensor Failure** (e.g., glass surface causes false readings, sensor hardware failure):
   - **Detection**: PointCloud2 messages contain all NaN values, or range readings implausible (<0.1m when robot is not near walls)
   - **Degradation Strategy**: Use camera stereo vision for depth estimation (if stereo cameras available), or rely on IMU + known map for collision-free planning
   - **Performance Impact**: Reduced obstacle detection range, cannot safely approach unknown objects

3. **IMU Drift** (e.g., bias accumulation over 10+ minutes):
   - **Detection**: Orientation estimate diverges from visual horizon or gravity direction
   - **Degradation Strategy**: Re-initialize IMU with camera-based horizon detection or magnetometer reading
   - **Performance Impact**: Temporary disorientation during re-initialization, reduced balance control accuracy

4. **Sensor Synchronization Failure** (e.g., camera and depth sensor timestamps differ by >100 ms):
   - **Detection**: message_filters::ApproximateTimeSynchronizer fails to match messages
   - **Degradation Strategy**: Use sensors independently without fusion, or increase synchronization slop parameter
   - **Performance Impact**: Reduced fusion accuracy, potential mismatch between camera and depth data

5. **Complete Sensor Suite Failure** (unlikely but safety-critical):
   - **Detection**: All sensor topics silent for >2 seconds
   - **Degradation Strategy**: Emergency stop, audible alarm, wait for manual intervention
   - **Performance Impact**: Robot cannot operate autonomously

**Format**: Table with columns: Failure Mode | Detection Method | Degradation Strategy | Performance Impact

### 3.6 Performance Criteria and Constraints (75-100 words)

Students must specify measurable performance metrics:

**Latency**:
- Sensor-to-decision latency budget (e.g., "obstacle detection → path replanning in <200 ms")
- Contribution from each stage (e.g., camera capture 30 ms, network 10 ms, processing 100 ms, planning 60 ms)

**Accuracy**:
- Localization error tolerance (e.g., "±5 cm for warehouse shelf alignment, ±50 cm for outdoor delivery waypoints")
- Depth measurement accuracy (e.g., "±2 cm at 1m range for grasping")

**Update Rates**:
- Minimum required for each sensor (e.g., "camera 15 Hz for object recognition, 30 Hz for visual odometry, IMU 100 Hz for balance control")

**Computational Cost**:
- Estimate CPU/GPU load (e.g., "stereo depth matching 30% of GPU, object detection 40% of GPU, fusion 10% of CPU")
- Trade-off analysis: reducing resolution to fit compute budget

**Cost Budget**:
- Total sensor cost (must not exceed scenario constraint)
- Per-sensor breakdown (e.g., "LiDAR $8,000, cameras $1,500, IMU $500 = $10,000 total")

**Estimated Length**: 500-600 words total for Section 3

---

## Section 4: Deliverables

**Purpose**: Specify exactly what students must submit

**Planned Content**:

### 4.1 Design Document Structure (150-175 words)

**Format**: PDF or Markdown, 2-3 pages (excluding diagrams)

**Required Sections**:

1. **Scenario Selection and Justification** (0.25 pages):
   - Which scenario chosen (A, B, C, or D)
   - Why this scenario is interesting or relevant to student
   - Brief overview of robot's primary task

2. **Sensor Specifications** (0.5 pages):
   - Table: Sensor Type | Model Example | Specifications | Placement | Update Rate | Cost
   - Justification paragraph for each sensor choice (camera, depth, IMU)

3. **ROS2 Node Architecture Diagram** (0.5 pages):
   - Visual diagram (hand-drawn or digital tool)
   - Labels for all nodes, topics, message types
   - Brief description of data flow (1-2 paragraphs)

4. **Sensor Fusion Strategy** (0.5 pages):
   - Fusion algorithm selection with rationale
   - Block diagram showing sensor inputs → fusion algorithm → state outputs
   - Explanation of which sensors contribute to which state variables (2-3 paragraphs)

5. **Failure Mode Analysis** (0.5 pages):
   - Table with 3-5 failure scenarios (format from Section 3.5)
   - Brief discussion of most critical failure and mitigation (1 paragraph)

6. **Trade-off Justifications** (0.25 pages):
   - Summary of 2-3 major design trade-offs
   - Examples: "Chose 640×480 over 1080p to reduce latency from 150ms to 50ms, acceptable loss of detail for navigation task"

### 4.2 Optional Code Sketch (50-75 words)

**Not required to run**, but students may include:

- ROS2 node pseudocode or Python skeleton showing:
  - Subscription setup to multiple sensors
  - message_filters for sensor synchronization
  - Fusion logic structure (commented outline, no full implementation)

**Example**:
```python
class SensorFusionNode(Node):
    def __init__(self):
        # Subscribe to camera, depth, IMU
        # Setup ApproximateTimeSynchronizer
        pass

    def fusion_callback(self, image, depth, imu):
        # Fuse data (placeholder for EKF update)
        pass
```

**Purpose**: Demonstrates understanding of ROS2 integration concepts from Module 2, but full implementation not expected

### 4.3 Submission Guidelines (50-75 words)

**File Naming**: `module2_capstone_[StudentName].pdf` or `.md`

**Submission Method**: Upload to course platform (LMS, GitHub, etc.)

**Late Policy**: Reference course syllabus

**Academic Integrity**: Students may discuss scenarios and general approaches, but sensor selection and architecture design must be individual work. Using AI tools (Claude, ChatGPT) for brainstorming or reviewing is encouraged; copying designs from online sources without attribution is not permitted.

**Estimated Length**: 225-275 words total for Section 4

---

## Section 5: Success Criteria

**Purpose**: Define what constitutes a successful capstone design

**Planned Content**:

### 5.1 Technical Completeness (100-125 words)

Design is complete if:

- **All 4 lesson concepts applied**:
  - Camera system selected with resolution, FOV, frame rate specified (Lesson 1)
  - Depth sensor selected with range, accuracy, technology type specified (Lesson 2)
  - IMU selected with update rate and range specified (Lesson 3)
  - Fusion algorithm chosen and justified (Lesson 4)

- **ROS2 integration detailed**:
  - Message types correctly referenced (sensor_msgs/Image, PointCloud2, LaserScan, Imu)
  - Topic names follow ROS2 conventions (e.g., `/camera/image_raw`, `/imu/data`)
  - Node architecture shows clear data flow from sensors → processing → fusion

- **All deliverable sections present**: Scenario, sensors, ROS2 diagram, fusion strategy, failure modes, trade-offs

### 5.2 Technical Justification (100-125 words)

Design is well-justified if:

- **Sensor choices reference specific trade-offs from Lessons 1-4**:
  - Example: "Chose stereo camera over RGB-D because scenario B requires outdoor operation where sunlight interferes with IR-based depth sensors (Lesson 2, Section 3.2)"
  - NOT acceptable: "Chose camera because it's good"

- **Fusion algorithm matches sensor characteristics**:
  - Example: "VIO (Visual-Inertial Odometry) appropriate for camera + IMU, not LiDAR + magnetometer"
  - Complementary filter for simple IMU + magnetometer orientation fusion
  - EKF for camera + IMU position/orientation fusion
  - robot_localization for multi-sensor fusion with GPS

- **Failure modes demonstrate sensor limitations from lessons**:
  - Example: "RGB-D depth sensor fails on glass surfaces (Lesson 2)" → "Fallback to stereo camera depth estimation"

### 5.3 Architectural Rigor (75-100 words)

Design shows professional-level thinking if:

- **ROS2 architecture is realistic and implementable**:
  - Topic names and message types are correct for ROS2 Humble
  - QoS policies appropriate for sensor type (best effort for cameras, reliable for critical control)
  - Synchronization mechanism specified (message_filters, ApproximateTimeSynchronizer)

- **Performance criteria are quantified**:
  - Latency budgets with breakdown (capture, network, processing)
  - Accuracy targets (±X cm for localization, ±Y cm for depth)
  - Update rate requirements (X Hz for camera, Y Hz for IMU)

- **Cost budget is realistic**:
  - Total sensor cost does not exceed scenario constraint
  - Acknowledges trade-offs (e.g., "Could use $20k LiDAR for better range but exceeds budget, chose $8k model")

### 5.4 Failure Analysis Depth (50-75 words)

Design demonstrates understanding of robustness if:

- **Failure modes are specific, not generic**:
  - Good: "Camera overexposure in direct sunlight causes saturation, detected by checking if >95% pixels have value 255"
  - Bad: "Camera might fail"

- **Degradation strategies are actionable**:
  - Good: "Fall back to depth sensor + dead reckoning, disable object recognition module, reduce max speed from 1.5 m/s to 0.5 m/s"
  - Bad: "Use backup sensor"

- **Critical failures identified**: Emergency stop scenario when all sensors fail

**Estimated Length**: 300-350 words total for Section 5

---

## Section 6: Resources and References

**Purpose**: Provide links to Module 2 lessons and external references

**Planned Content**:

### 6.1 Module 2 Lessons (75-100 words)

Review these lessons before starting your design:

- [Lesson 1: Camera Systems and Computer Vision](./01-camera-systems) - Camera types (monocular, stereo, RGB-D), resolution, FOV, ROS2 sensor_msgs/Image
- [Lesson 2: Depth Sensing Technologies](./02-depth-sensing) - LiDAR, structured light, ToF, sensor_msgs/PointCloud2, LaserScan, outdoor vs. indoor trade-offs
- [Lesson 3: IMU and Proprioception](./03-imu-proprioception) - Accelerometer, gyroscope, magnetometer, sensor_msgs/Imu, balance control, drift
- [Lesson 4: Sensor Fusion Techniques](./04-sensor-fusion) - Complementary filter, Kalman filter, VIO, robot_localization package, multi-sensor integration

### 6.2 Case Studies (75-100 words)

Real-world examples for inspiration:

- **Boston Dynamics Atlas**: Multi-sensor SLAM with LiDAR + stereo cameras + IMU for navigation in disaster environments
- **Agility Robotics Digit**: VIO using stereo cameras + IMU for GPS-free warehouse navigation
- **Tesla Optimus**: Multi-camera fusion (8+ cameras) for manipulation and navigation, no LiDAR
- **ANYbotics ANYmal**: LiDAR + stereo + IMU fusion for outdoor legged robot navigation on rough terrain

**Note**: Use these as conceptual references, not blueprints. Your design should be tailored to your scenario constraints.

### 6.3 ROS2 Documentation (50-75 words)

Technical references for implementation details:

- **robot_localization package**: [ROS2 Humble documentation](http://docs.ros.org/en/humble/p/robot_localization/) - EKF for multi-sensor fusion
- **message_filters**: [ROS2 ApproximateTimeSynchronizer](http://docs.ros.org/en/humble/p/message_filters/) - Synchronizing camera + depth + IMU
- **sensor_msgs**: [Message type definitions](http://docs.ros.org/en/humble/p/sensor_msgs/) - Image, PointCloud2, LaserScan, Imu, CameraInfo

### 6.4 Design Tools (50-75 words)

Suggested tools for creating diagrams:

- **draw.io** (diagrams.net): Free web-based diagramming tool, supports ROS2 node/topic shapes
- **ROS2 rqt_graph**: Generates real-time ROS2 architecture diagrams (if you have ROS2 installed)
- **Lucidchart**: Web-based diagramming (free tier available)
- **Paper and pencil**: Hand-drawn diagrams are fully acceptable if legible

**Estimated Length**: 225-275 words total for Section 6

---

## Section 7: Expert Tips and AI Colearning

**Purpose**: Provide professional context and AI-assisted learning prompts

**Planned Content**:

### 💬 AI Colearning Prompt 1: Scenario Exploration (75-100 words)

> **Before starting your design, ask your AI assistant**:
>
> "I'm designing a sensor system for [your chosen scenario]. Compare the trade-offs between using:
> 1. Stereo cameras + IMU (passive depth, lower cost)
> 2. LiDAR + monocular camera + IMU (active depth, higher cost)
>
> Consider: indoor vs. outdoor operation, sunlight interference, compute requirements, and cost. Which sensor suite would you recommend and why?"
>
> **Learning outcome**: This prompt helps you explore sensor trade-offs specific to your scenario before committing to a design. Compare the AI's reasoning with concepts from Lessons 1-2.

### 💬 AI Colearning Prompt 2: Fusion Strategy Validation (75-100 words)

> **After drafting your fusion strategy, ask your AI assistant**:
>
> "I'm using [your fusion algorithm] to combine [your sensors]. My state variables are [position, orientation, velocity, etc.]. Which sensors should contribute to which state variables, and why? Are there any sensor combinations that don't make sense?"
>
> **Learning outcome**: This prompt helps validate that your fusion approach matches sensor characteristics. For example, cameras don't directly measure velocity, but can estimate it via frame-to-frame motion. IMUs measure acceleration/angular velocity, not position directly.

### 🎓 Expert Insight: Real-World Iteration Process (100-125 words)

**Sensor Systems Are Never Perfect on First Try**

Professional robotics teams iterate sensor configurations 5-10 times before finalizing designs. Boston Dynamics' Atlas went through multiple camera/LiDAR combinations before settling on its current sensor suite. Tesla Optimus initially tested with LiDAR before switching to a camera-only approach.

**Your capstone design won't be perfect, and that's expected.** The goal is to demonstrate:
1. **Explicit reasoning**: Why did you choose sensor X over Y? What trade-off did you prioritize?
2. **Awareness of limitations**: What fails in your design, and how do you mitigate it?
3. **Realistic constraints**: Cost, compute, latency budgets force hard choices—that's engineering.

**Grading prioritizes justification quality over "optimal" sensor selection.** Two students can choose different sensors for the same scenario and both earn full marks if their reasoning is sound.

### 🤝 Practice Exercise: ROS2 Message Synchronization (100-125 words)

**Challenge**: Sketch the ROS2 `message_filters.ApproximateTimeSynchronizer` setup for your sensor suite.

Consider:
1. **Which sensors need synchronization?** (e.g., camera + depth for 3D object detection, or camera + IMU for VIO)
2. **What slop parameter would you use?** (Maximum time difference between messages to consider "synchronized")
   - Example: Camera at 15 Hz (66 ms period), IMU at 100 Hz (10 ms period) → slop ~50 ms to allow 1-2 camera frames of drift
3. **What happens if synchronization fails?** (e.g., one sensor stops publishing)

**Pseudocode template**:
```python
sync = ApproximateTimeSynchronizer(
    [camera_sub, depth_sub, imu_sub],
    queue_size=10,
    slop=0.05  # 50 ms tolerance
)
sync.registerCallback(fusion_callback)
```

**Optional**: Ask Claude to review your slop parameter choice and suggest improvements based on your sensor update rates.

**Estimated Length**: 325-375 words total for Section 7

---

## Outline Validation Checklist

### Structure Compliance
- [x] All required outline sections present (Overview, 7 content sections)
- [x] 4 distinct scenarios (A: Home Assistant, B: Outdoor Delivery, C: Human Interaction, D: Warehouse)
- [x] Each scenario requires all 4 lesson concepts (camera, depth, IMU, fusion)
- [x] Design requirements detailed (sensor selection, placement, ROS2, fusion, failure modes, performance)
- [x] Deliverables specified (document structure, optional code, submission)
- [x] Success criteria defined (completeness, justification, architecture, failure analysis)
- [x] Resources include Docusaurus internal links and external references
- [x] Expert tips with AI colearning prompts

### Scenario Coverage Validation

**Scenario A (Home Assistant)**:
- [x] Camera: Object recognition, visual servoing, localization (L1) ✓
- [x] Depth: Obstacle avoidance, grasp distance, surface detection (L2) ✓
- [x] IMU: Balance control, fall detection, motion prediction (L3) ✓
- [x] Fusion: VIO for indoor localization, obstacle+balance fusion (L4) ✓

**Scenario B (Outdoor Delivery)**:
- [x] Camera: Semantic navigation, visual landmarks, texture analysis (L1) ✓
- [x] Depth: Long-range LiDAR, terrain mapping, curb detection (L2) ✓
- [x] IMU: Tilt on slopes, vibration damping, load sensing (L3) ✓
- [x] Fusion: LiDAR+IMU SLAM, GPS+VIO, multi-modal obstacles (L4) ✓

**Scenario C (Human Interaction)**:
- [x] Camera: Face tracking, gesture recognition, handoff servoing (L1) ✓
- [x] Depth: Safe approach distance, 3D person tracking, obstacle detection (L2) ✓
- [x] IMU: Head stabilization, gentle motion, collision reaction (L3) ✓
- [x] Fusion: Camera+depth 3D tracking, IMU+vision head stabilization (L4) ✓

**Scenario D (Warehouse)**:
- [x] Camera: Barcode scanning, multi-camera odometry, shelf edges (L1) ✓
- [x] Depth: High-speed LiDAR obstacles, shelf localization, item dimensions (L2) ✓
- [x] IMU: Rapid acceleration, high-speed balance, vibration filtering (L3) ✓
- [x] Fusion: Multi-camera VIO, LiDAR+Visual SLAM, IMU stabilization (L4) ✓

### Pedagogical Quality
- [x] Scenarios are distinct (home vs. outdoor vs. social vs. warehouse)
- [x] Technical challenges specific to each scenario (lighting, terrain, human safety, speed)
- [x] Design requirements mirror professional practice (justification, architecture, failure analysis)
- [x] Success criteria emphasize reasoning quality over "correct" answers
- [x] AI colearning prompts encourage exploration before and validation during design
- [x] Expert insight sets realistic expectations (iteration is normal)

### ROS2 Integration
- [x] Message types correctly referenced (sensor_msgs/Image, PointCloud2, LaserScan, Imu, CameraInfo, NavSatFix, Odometry)
- [x] Topic naming conventions follow ROS2 standards (/camera/image_raw, /imu/data, /scan, etc.)
- [x] Node architecture guidance includes drivers, processing, fusion nodes
- [x] QoS considerations mentioned (best effort vs. reliable)
- [x] message_filters synchronization explained with slop parameter
- [x] robot_localization package referenced for EKF fusion

### Estimated Word Counts
- Section 1 (Introduction): 250-300 words ✓
- Section 2 (Scenarios): 350-400 words × 4 = 1,400-1,600 words ✓
- Section 3 (Design Requirements): 500-600 words ✓
- Section 4 (Deliverables): 225-275 words ✓
- Section 5 (Success Criteria): 300-350 words ✓
- Section 6 (Resources): 225-275 words ✓
- Section 7 (Expert Tips): 325-375 words ✓

**Total Estimated**: 3,225-3,775 words for outline (will guide ~1,800-2,200 word final content)

### Links and References
- [x] Internal Docusaurus links use relative paths (./01-camera-systems, ./02-depth-sensing, etc.)
- [x] External links provided for ROS2 documentation (docs.ros.org)
- [x] Case studies cited (Atlas, Digit, Optimus, ANYmal)
- [x] Tools referenced (draw.io, rqt_graph, Lucidchart)

---

## Summary for Content Writer Agent

**Outline Status**: Complete and ready for capstone content development

**Key Points for Expansion**:

1. **Scenario Differentiation**: Ensure each scenario's technical challenges are distinct and clearly tied to lesson concepts:
   - Scenario A: Indoor lighting variability, reflective surfaces (glass/mirrors)
   - Scenario B: Outdoor sunlight, long-range detection, terrain variation
   - Scenario C: Human safety, privacy, close-range precision
   - Scenario D: High-speed latency, harsh lighting, precise localization

2. **Design Requirement Depth**: Expand sensor specification section with:
   - Example model numbers (Intel RealSense D435, Velodyne Puck, MPU-9250)
   - Comparison tables (resolution vs. cost, range vs. accuracy)
   - Explicit trade-off examples ("640×480 at 30 FPS vs. 1080p at 15 FPS")

3. **ROS2 Architecture Precision**: Ensure all message types, topic names, and node structures are accurate for ROS2 Humble:
   - Verify sensor_msgs package message types
   - Include QoS policy recommendations
   - Reference robot_localization EKF parameters

4. **Failure Mode Realism**: Expand failure scenarios with:
   - Specific detection methods (e.g., "check PointCloud2 for NaN values")
   - Quantified degradation strategies (e.g., "reduce max speed from 1.5 m/s to 0.5 m/s")
   - Safety-critical considerations (emergency stop logic)

5. **Grading Clarity**: Emphasize in success criteria that:
   - Multiple sensor configurations can be valid for same scenario
   - Justification quality matters more than "optimal" choice
   - Explicit trade-off reasoning is required (not just listing specs)

**Estimated Total Word Count**: 1,800-2,200 words (final content will be more concise than outline)

**Diagrams/Visuals Needed**:
- 4 scenario illustrations (simple humanoid robot in context: home, campus, hospital, warehouse)
- Sensor placement diagram template (humanoid with labeled mount points)
- ROS2 architecture diagram example (nodes, topics, message types)
- Sensor fusion block diagram template (sensors → fusion → state estimate)
- Failure mode decision tree (sensor failure → detection → degradation)

**External References to Verify**:
- ROS2 Humble sensor_msgs package documentation (message type definitions)
- robot_localization package EKF configuration examples
- message_filters ApproximateTimeSynchronizer API (slop parameter)
- Boston Dynamics Atlas, Agility Robotics Digit, Tesla Optimus sensor configurations (from public documentation)

**Special Considerations**:
- **No coding required**: Emphasize design document only, code sketch is optional
- **AI tool encouragement**: Students should use Claude/ChatGPT for brainstorming and validation
- **Privacy note**: Scenario C requires addressing data privacy (edge processing, no persistent storage)
- **Cost realism**: Ensure sensor budgets reflect real market prices (e.g., LiDAR $5k-$20k, cameras $100-$2k, IMU $50-$2k)

---

**Next Steps**:
- Content Writer Agent: Expand outline into full capstone project content
- Technical Reviewer Agent: Verify ROS2 message types, sensor specifications, and fusion algorithms
- Diagram Artist (if available): Create visual aids for scenarios, sensor placement, ROS2 architecture
