# Data Model: Module 2 - Sensors and Perception for Humanoid Robots

**Date**: 2025-12-07
**Feature**: 002-sensors-perception
**Purpose**: Define entities, schemas, and validation rules for Module 2 content

## Overview

This document defines the data structures, relationships, and validation rules for Module 2 educational content. Unlike software projects with databases and APIs, this "data model" describes the structure of educational entities (lessons, sensors, ROS2 messages) and their metadata schemas.

---

## Entity Definitions

### 1. Lesson

**Description**: A single educational content unit (markdown file) covering one aspect of sensor perception.

#### Fields

| Field | Type | Required | Validation | Purpose |
|-------|------|----------|------------|---------|
| `title` | string | ✅ | Max 60 chars, descriptive | Lesson heading and page title |
| `sidebar_position` | integer | ✅ | 1-6 (lessons 01-04, capstone 05, quiz 06) | Docusaurus sidebar ordering |
| `skills` | array<Skill> | ✅ | 1-3 skills per lesson | Learning competencies |
| `learning_objectives` | array<LearningObjective> | ✅ | 3-5 objectives per lesson | Measurable outcomes |
| `cognitive_load` | CognitiveLoad | ✅ | Object with new_concepts + assessment | Learning complexity |
| `differentiation` | Differentiation | ✅ | Object with extension + remedial | Adaptive learning paths |
| `tags` | array<string> | ✅ | 3-5 tags, lowercase, hyphenated | Searchability, RAG metadata |
| `generated_by` | string | ✅ | "agent" or "human" | Authorship tracking |
| `created` | string | ✅ | ISO 8601 date (YYYY-MM-DD) | Creation timestamp |
| `last_modified` | string | ✅ | ISO 8601 date (YYYY-MM-DD) | Update timestamp |

#### Relationships

- **Has One**: Paired `.summary.md` file (1:1 relationship)
- **Belongs To**: Module (many lessons : 1 module)
- **Contains**: 0-3 code examples (embedded in markdown)
- **References**: 0-4 images (SVG diagrams in `/static/img/`)

#### State Transitions

```
Draft → Technical Review → Style Review → Approved → Published
  ↓           ↓                 ↓            ↓          ↓
(Agent 1-4) (Agent 5)        (Agent 6)   (Agent 7-8) (User)
```

#### Validation Rules

1. **Frontmatter Completeness**: All 13 fields present (no optional fields)
2. **Sidebar Position Uniqueness**: No two lessons in same module share sidebar_position
3. **Summary File Existence**: For every `NN-lesson-name.md`, must have `NN-lesson-name.summary.md`
4. **Tag Consistency**: Tags must match module-level taxonomy (from constitution)
5. **Learning Objective Measurability**: Each objective must have `assessment_method` defined

---

### 2. Skill

**Description**: A discrete competency or ability students acquire from a lesson.

#### Schema

```yaml
- name: string                      # Skill name (e.g., "ROS2 Sensor Messages")
  proficiency_level: enum           # beginner | intermediate | advanced
  category: string                  # Taxonomy category (e.g., "robotics-middleware")
  bloom_level: enum                 # remember | understand | apply | analyze | evaluate | create
  digcomp_area: string              # Digital Competence Framework area
  measurable_at_this_level: string  # How skill is demonstrated (e.g., "subscribe to sensor_msgs/Image topic")
```

#### Validation Rules

- `proficiency_level` ∈ {beginner, intermediate, advanced}
- `bloom_level` ∈ {remember, understand, apply, analyze, evaluate, create}
- `measurable_at_this_level`: Must be action-oriented (verb + object)

#### Module 2 Skill Examples

**Lesson 1 (Camera Systems)**:
```yaml
- name: "Camera Types for Robotics"
  proficiency_level: "beginner"
  category: "sensor-perception"
  bloom_level: "understand"
  digcomp_area: "technical-concepts"
  measurable_at_this_level: "differentiate monocular, stereo, and RGB-D cameras"
```

**Lesson 2 (Depth Sensing)**:
```yaml
- name: "Point Cloud Processing"
  proficiency_level: "beginner"
  category: "sensor-perception"
  bloom_level: "apply"
  digcomp_area: "data-processing"
  measurable_at_this_level: "interpret sensor_msgs/PointCloud2 structure"
```

**Lesson 3 (IMU)**:
```yaml
- name: "IMU-Based Orientation Estimation"
  proficiency_level: "intermediate"
  category: "sensor-fusion"
  bloom_level: "analyze"
  digcomp_area: "problem-solving"
  measurable_at_this_level: "explain gyroscope drift and mitigation strategies"
```

**Lesson 4 (Sensor Fusion)**:
```yaml
- name: "Multi-Sensor Integration"
  proficiency_level: "intermediate"
  category: "sensor-fusion"
  bloom_level: "apply"
  digcomp_area: "system-design"
  measurable_at_this_level: "design fusion strategy for visual-inertial odometry"
```

---

### 3. LearningObjective

**Description**: A measurable outcome students achieve after completing a lesson.

#### Schema

```yaml
- objective: string                 # What students will learn (e.g., "Understand ROS2 Image messages")
  proficiency_level: enum           # beginner | intermediate | advanced
  bloom_level: enum                 # remember | understand | apply | analyze | evaluate | create
  assessment_method: string         # How objective is measured (e.g., "quiz question 3-5")
```

#### Validation Rules

- `objective`: Must start with Bloom verb (Understand, Apply, Analyze, etc.)
- `proficiency_level` matches lesson's target level
- `bloom_level` aligns with objective verb
- `assessment_method`: References specific quiz questions or capstone tasks

#### Module 2 Learning Objective Examples

**Lesson 1**:
```yaml
- objective: "Understand the differences between monocular, stereo, and RGB-D cameras"
  proficiency_level: "beginner"
  bloom_level: "understand"
  assessment_method: "quiz questions 1-2, capstone sensor selection"
```

**Lesson 2**:
```yaml
- objective: "Analyze trade-offs between LiDAR and structured light depth sensors"
  proficiency_level: "intermediate"
  bloom_level: "analyze"
  assessment_method: "quiz question 7, capstone justification"
```

---

### 4. CognitiveLoad

**Description**: Quantifies the learning difficulty and mental effort required for a lesson.

#### Schema

```yaml
cognitive_load:
  new_concepts: integer             # Number of new concepts introduced (3-8)
  assessment: string                # low | moderate | high (with justification)
```

#### Validation Rules

- `new_concepts`: 3-8 (too few = trivial, too many = overwhelming)
- `assessment`: Must include justification (e.g., "moderate - builds on Module 1 ROS2 knowledge")

#### Module 2 Cognitive Load Examples

**Lesson 1** (Camera Systems):
```yaml
cognitive_load:
  new_concepts: 5  # monocular, stereo, RGB-D, sensor_msgs/Image, CameraInfo
  assessment: "moderate - builds on ROS2 topic understanding from Module 1"
```

**Lesson 3** (IMU):
```yaml
cognitive_load:
  new_concepts: 6  # accelerometer, gyroscope, magnetometer, drift, proprioception, sensor_msgs/Imu
  assessment: "moderate-high - introduces new physics concepts (inertial sensing)"
```

---

### 5. Differentiation

**Description**: Adaptive learning paths for students with different needs.

#### Schema

```yaml
differentiation:
  extension_for_advanced: string    # Challenge for fast learners
  remedial_for_struggling: string   # Support for students needing help
```

#### Validation Rules

- Both fields required (no empty strings)
- `extension_for_advanced`: Suggests deeper exploration (research papers, advanced tools)
- `remedial_for_struggling`: Points to prerequisite review or simpler examples

#### Module 2 Differentiation Examples

**Lesson 1**:
```yaml
differentiation:
  extension_for_advanced: "Explore camera calibration mathematics (Zhang's method) and distortion correction"
  remedial_for_struggling: "Review image representation basics (pixels, RGB encoding) before ROS2 messages"
```

**Lesson 4**:
```yaml
differentiation:
  extension_for_advanced: "Implement Extended Kalman Filter for visual-inertial odometry in Python"
  remedial_for_struggling: "Review Lesson 3 (IMU concepts) and focus on complementary filter intuition"
```

---

### 6. Sensor Type

**Description**: Physical sensor hardware with measurable characteristics.

#### Attributes

| Attribute | Type | Example Values | Purpose |
|-----------|------|----------------|---------|
| `name` | string | "RGB Camera", "2D LiDAR", "9-DOF IMU" | Human-readable identifier |
| `category` | enum | camera \| depth \| imu \| tactile \| audio | Sensor modality |
| `data_format` | string | "sensor_msgs/Image", "sensor_msgs/LaserScan" | ROS2 message type |
| `update_rate` | integer | 30 (Hz) | Measurement frequency |
| `range` | string | "0.5-10m", "N/A" | Effective sensing distance |
| `accuracy` | string | "±3cm", "±1°" | Measurement precision |
| `failure_modes` | array<string> | ["low-light", "motion blur", "occlusion"] | Known limitations |

#### Module 2 Sensor Type Catalog

**Cameras**:
1. Monocular Camera: {category: "camera", data_format: "sensor_msgs/Image", update_rate: 30, range: "N/A", failure_modes: ["low-light", "motion blur"]}
2. Stereo Camera: {category: "camera", data_format: "sensor_msgs/Image" (2 topics), update_rate: 30, range: "0.5-10m", failure_modes: ["textureless surfaces", "calibration drift"]}
3. RGB-D Camera: {category: "camera", data_format: "sensor_msgs/Image + PointCloud2", update_rate: 30, range: "0.3-4m", failure_modes: ["outdoor IR interference", "transparent objects"]}

**Depth Sensors**:
1. 2D LiDAR: {category: "depth", data_format: "sensor_msgs/LaserScan", update_rate: 40, range: "0.1-30m", accuracy: "±3cm", failure_modes: ["transparent surfaces", "specular reflections"]}
2. 3D LiDAR: {category: "depth", data_format: "sensor_msgs/PointCloud2", update_rate: 10, range: "10-100m", accuracy: "±2cm", failure_modes: ["rain/snow interference"]}
3. Structured Light Depth: {category: "depth", data_format: "sensor_msgs/PointCloud2", update_rate: 30, range: "0.5-4m", accuracy: "±1cm", failure_modes: ["outdoor use", "IR crosstalk"]}

**IMU**:
1. 6-DOF IMU: {category: "imu", data_format: "sensor_msgs/Imu", update_rate: 100, accuracy: "±0.5° (orientation)", failure_modes: ["gyroscope drift", "vibration noise"]}
2. 9-DOF IMU: {category: "imu", data_format: "sensor_msgs/Imu", update_rate: 200, accuracy: "±0.3°", failure_modes: ["magnetic interference", "accelerometer bias"]}

---

### 7. ROS2 Message Type

**Description**: Standard ROS2 message definition for sensor data exchange.

#### Attributes

| Attribute | Type | Example | Purpose |
|-----------|------|---------|---------|
| `package` | string | "sensor_msgs", "geometry_msgs" | ROS2 package namespace |
| `message_name` | string | "Image", "PointCloud2", "Imu" | Message type name |
| `fields` | array<string> | ["header", "height", "width", "data"] | Message structure |
| `purpose` | string | "Transmit camera image data" | Use case description |

#### Module 2 ROS2 Message Catalog

**sensor_msgs/Image**:
```yaml
package: "sensor_msgs"
message_name: "Image"
fields:
  - "header: std_msgs/Header"
  - "height: uint32"
  - "width: uint32"
  - "encoding: string"
  - "is_bigendian: uint8"
  - "step: uint32"
  - "data: uint8[]"
purpose: "Transmit uncompressed image data from cameras"
typical_topics: ["/camera/image_raw", "/camera/rgb/image", "/head_camera/image"]
```

**sensor_msgs/PointCloud2**:
```yaml
package: "sensor_msgs"
message_name: "PointCloud2"
fields:
  - "header: std_msgs/Header"
  - "height: uint32"
  - "width: uint32"
  - "fields: PointField[]"
  - "is_bigendian: bool"
  - "point_step: uint32"
  - "row_step: uint32"
  - "data: uint8[]"
  - "is_dense: bool"
purpose: "Transmit 3D point cloud data from LiDAR or depth cameras"
typical_topics: ["/lidar/points", "/depth_camera/points", "/scan/cloud"]
```

**sensor_msgs/Imu**:
```yaml
package: "sensor_msgs"
message_name: "Imu"
fields:
  - "header: std_msgs/Header"
  - "orientation: geometry_msgs/Quaternion"
  - "orientation_covariance: float64[9]"
  - "angular_velocity: geometry_msgs/Vector3"
  - "angular_velocity_covariance: float64[9]"
  - "linear_acceleration: geometry_msgs/Vector3"
  - "linear_acceleration_covariance: float64[9]"
purpose: "Transmit IMU sensor data (orientation, angular velocity, acceleration)"
typical_topics: ["/imu/data", "/torso_imu", "/head_imu/raw"]
```

---

### 8. Perception Concept

**Description**: Abstract concept or metric relevant to sensor perception.

#### Attributes

| Attribute | Type | Example | Purpose |
|-----------|------|---------|---------|
| `name` | string | "Field of View (FOV)" | Concept identifier |
| `definition` | string | "Angular extent of observable world" | Plain-language explanation |
| `relevance_to_humanoids` | string | "Wider FOV enables better peripheral awareness" | Application context |
| `measurable_characteristic` | string | "degrees (horizontal × vertical)" | Quantifiable metric |

#### Module 2 Perception Concept Catalog

1. **Field of View (FOV)**:
   - Definition: "Angular extent of the observable world captured by a camera"
   - Relevance: "Humanoid navigation requires wide FOV (120°+) for obstacle awareness; manipulation needs narrow FOV (60°) for precision"
   - Measurable: "degrees (e.g., 90° horizontal × 60° vertical)"

2. **Point Cloud Density**:
   - Definition: "Number of 3D points captured per unit area"
   - Relevance: "Higher density enables precise object shape reconstruction for grasping"
   - Measurable: "points/m² or total points/scan"

3. **IMU Drift**:
   - Definition: "Accumulation of orientation error over time due to gyroscope bias"
   - Relevance: "Unbounded drift makes long-term IMU-only orientation estimation impossible; requires sensor fusion"
   - Measurable: "degrees/minute or degrees/hour"

4. **Sensor Latency**:
   - Definition: "Time delay between physical event and sensor measurement availability"
   - Relevance: "High latency (>100ms) causes delayed reactions in humanoid balance control"
   - Measurable: "milliseconds (ms)"

5. **Depth Accuracy**:
   - Definition: "Precision of distance measurements from depth sensors"
   - Relevance: "Manipulation requires high accuracy (±1cm); navigation tolerates lower accuracy (±5cm)"
   - Measurable: "±Xcm or ±X% of range"

---

### 9. Fusion Strategy

**Description**: Algorithmic approach to combining multiple sensor inputs.

#### Attributes

| Attribute | Type | Example | Purpose |
|-----------|------|---------|---------|
| `name` | string | "Visual-Inertial Odometry" | Strategy name |
| `sensors_combined` | array<string> | ["camera", "IMU"] | Input sensors |
| `technique` | string | "Extended Kalman Filter" | Algorithm/method |
| `use_case` | string | "Robot localization indoors" | Application scenario |
| `ros2_implementation` | string | "robot_localization package" | Existing ROS2 tool (if available) |

#### Module 2 Fusion Strategy Catalog

1. **Visual-Inertial Odometry (VIO)**:
   - Sensors: ["monocular camera", "IMU"]
   - Technique: "Extended Kalman Filter (EKF) tracking visual features + IMU prediction"
   - Use Case: "Humanoid robot localization in GPS-denied indoor environments"
   - ROS2: "rtabmap_ros, ORB_SLAM3 (with ROS2 wrappers)"

2. **Depth-Enhanced Object Detection**:
   - Sensors: ["RGB camera", "depth camera"]
   - Technique: "Early fusion (concatenate RGB+depth before CNN) or late fusion (detect in RGB, filter by depth)"
   - Use Case: "Humanoid manipulation - detect graspable objects and measure distance"
   - ROS2: "Custom perception nodes, MoveIt integration"

3. **IMU-Corrected Stereo Odometry**:
   - Sensors: ["stereo camera", "IMU"]
   - Technique: "Complementary filter (high-pass gyro + low-pass visual odometry)"
   - Use Case: "Outdoor humanoid navigation on uneven terrain"
   - ROS2: "robot_localization EKF with visual odometry + IMU topics"

4. **Multi-Sensor SLAM**:
   - Sensors: ["LiDAR", "camera", "IMU", "wheel odometry"]
   - Technique: "Graph-based SLAM with loop closure detection"
   - Use Case: "Humanoid environment mapping for path planning"
   - ROS2: "slam_toolbox, Cartographer"

---

## Validation Checklist

### Lesson Validation

- [ ] All 13 frontmatter fields present and correctly typed
- [ ] sidebar_position is unique within module
- [ ] Paired `.summary.md` file exists
- [ ] Learning objectives use Bloom verbs
- [ ] Code examples follow Python 3.11+ style (type hints)
- [ ] ROS2 message types referenced correctly (e.g., sensor_msgs/Image, not sensor_msg/Image)
- [ ] Images referenced exist in `/static/img/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/`
- [ ] No broken internal links (Docusaurus build succeeds)

### Module Validation

- [ ] 6 total files: 4 lessons + capstone + quiz
- [ ] Each lesson file has paired summary (12 total markdown files)
- [ ] Module README exists with navigation links
- [ ] All lessons follow 7-section content structure (What Is, Why Matters, Key Principles, Example, Exercise, Summary, Next Steps)
- [ ] Quiz references all 4 lessons (questions distributed across topics)
- [ ] Capstone integrates concepts from all 4 lessons

### Metadata Consistency

- [ ] Tags consistent across module (all lessons include "ros2", "sensors", "perception")
- [ ] Skill categories use controlled vocabulary (sensor-perception, sensor-fusion, robotics-middleware)
- [ ] Bloom levels progress logically (Lesson 1: understand, Lesson 4: apply/analyze)
- [ ] Cognitive load justified (module builds complexity: moderate → moderate-high → high)

---

## Summary

This data model defines:
- **9 entity types**: Lesson, Skill, LearningObjective, CognitiveLoad, Differentiation, SensorType, ROS2MessageType, PerceptionConcept, FusionStrategy
- **13-field frontmatter schema**: Enforced for all lesson files
- **Validation rules**: Ensure content quality, metadata consistency, and RAG-readiness
- **Catalog of 12 sensor types**: Cameras, depth sensors, IMUs with failure modes
- **Catalog of 4 ROS2 messages**: Image, PointCloud2, Imu, LaserScan with field structures
- **Catalog of 4 fusion strategies**: VIO, depth-enhanced detection, IMU-corrected odometry, multi-sensor SLAM

**Status**: ✅ Data model complete. Ready to generate contracts/ and quickstart.md.
