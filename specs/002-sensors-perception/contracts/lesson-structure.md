# Lesson Content Structure Contract

**Feature**: 002-sensors-perception
**Purpose**: Define the required structure and content patterns for all Module 2 lessons

## Overview

Every lesson in Module 2 (Sensors and Perception) MUST follow this 7-section structure to ensure consistency, RAG-readiness, and student learning progression. This contract specifies required sections, embedded callouts, code example standards, and validation criteria.

---

## Required 7-Section Pattern

### Section 1: What Is [Concept]?

**Purpose**: Clear definition and foundational understanding

**Requirements**:
- Start with a 1-2 sentence plain-language definition
- Explain the concept's role in humanoid robotics
- Introduce key terminology (bold important terms on first use)
- Length: 200-400 words

**Example Opening** (Lesson 1 - Camera Systems):
```markdown
## What Is a Camera in Robotics?

A **camera** is a sensor that captures visual information from the environment by converting light into digital images. In humanoid robotics, cameras serve as the primary means of perceiving the world, enabling tasks like object recognition, navigation, and human interaction.
```

**Validation**:
- [ ] Starts with H2 heading (`##`)
- [ ] Includes clear definition in first paragraph
- [ ] Defines 3-5 key terms (bolded)

---

### Section 2: Why [Concept] Matters for Physical AI

**Purpose**: Motivation and real-world relevance

**Requirements**:
- Connect concept to humanoid robot capabilities (manipulation, navigation, interaction)
- Provide 1-2 real-world examples (Boston Dynamics, Tesla Optimus, Agility Robotics)
- Explain what's impossible without this sensor/technique
- Length: 300-500 words

**Example** (Lesson 2 - Depth Sensing):
```markdown
## Why Depth Sensing Matters for Physical AI

While cameras provide rich visual information, they lack a critical dimension: **depth**. Without depth perception, a humanoid robot cannot determine whether an object is 1 meter or 10 meters away, making tasks like grasping, navigation, and obstacle avoidance nearly impossible.

Consider Tesla's Optimus robot sorting objects in a warehouse. The robot must not only *see* boxes but also *measure* their distance to plan a collision-free path and grasp them at the correct reach distance. Depth sensors like LiDAR and structured light cameras provide this 3D spatial awareness.
```

**Validation**:
- [ ] H2 heading with "Why ... Matters" format
- [ ] Includes at least one humanoid robot example
- [ ] Explains limitations without this concept

---

### Section 3: Key Principles

**Purpose**: Core concepts presented as enumerated list

**Requirements**:
- Use H3 subheading (`###`)
- 3-5 key principles or sub-concepts
- Each principle: 1-2 paragraphs explanation
- Include comparisons/trade-offs where relevant (e.g., LiDAR vs depth cameras)
- Optionally include a table for comparisons

**Example** (Lesson 3 - IMU):
```markdown
### Key Principles

#### 1. Three-Axis Sensing

IMUs measure motion along three perpendicular axes (X, Y, Z), providing complete 3D orientation and acceleration data. This 3-axis architecture mirrors how humans sense balance through the vestibular system in our inner ears.

#### 2. Complementary Sensor Types

A complete IMU combines three sensor types:
- **Accelerometer**: Measures linear acceleration (including gravity)
- **Gyroscope**: Measures rotational velocity (how fast the robot is turning)
- **Magnetometer**: Measures magnetic field direction (acts as a compass)

#### 3. Drift and the Need for Fusion

Gyroscopes suffer from **drift** - small measurement errors accumulate over time, causing the orientation estimate to diverge from reality. This is why IMUs are rarely used alone; they must be fused with other sensors (like cameras or magnetometers) for long-term accuracy.
```

**Validation**:
- [ ] H3 heading "Key Principles"
- [ ] 3-5 numbered sub-principles (H4 headings)
- [ ] At least one comparison/trade-off explained

---

### Section 4: Embedded Callouts

**Purpose**: Interactive learning prompts and expert insights

**Requirements**: Include 1-2 of each callout type per lesson

#### 💬 AI Colearning Prompt

**Format**:
```markdown
### 💬 AI Colearning Prompt

> **Ask your AI assistant**: "Explain how stereo cameras calculate depth using triangulation. Use a simple analogy involving human binocular vision."
>
> This prompt helps you explore the mathematical intuition behind stereo vision without diving into complex linear algebra.
```

**Purpose**: Encourages students to use Claude/ChatGPT for deeper exploration

**Validation**:
- [ ] Uses blockquote (`>`)
- [ ] Includes specific question for AI
- [ ] Explains what students will gain from the prompt

---

#### 🎓 Expert Insight

**Format**:
```markdown
### 🎓 Expert Insight

> **Common Pitfall**: Many beginners assume LiDAR works like a camera, producing a 2D image. In reality, LiDAR outputs a **point cloud** - an unordered set of 3D coordinates. You must process this raw data (filtering, clustering, segmentation) before extracting useful information like "there's a table 2 meters ahead."
```

**Purpose**: Warns students about common mistakes or shares advanced perspective

**Validation**:
- [ ] Uses blockquote
- [ ] Highlights a misconception, pitfall, or advanced tip

---

#### 🤝 Practice Exercise

**Format**:
```markdown
### 🤝 Practice Exercise

> **Challenge**: Open RViz and visualize a simulated camera's image feed from `/camera/image_raw`. Try changing the robot's camera position and observe how the image changes. Then, inspect the `/camera/camera_info` topic using `ros2 topic echo` - can you identify the camera's field of view from the `K` matrix?
>
> **Hint**: The focal length values in the intrinsic matrix `K` determine the field of view. Wider FOV = smaller focal length.
```

**Purpose**: Hands-on task applying the lesson concept

**Validation**:
- [ ] Uses blockquote
- [ ] Provides clear task instructions
- [ ] Optionally includes hint or success criteria

---

### Section 5: Practical Example (ROS2 Code)

**Purpose**: Demonstrate concept with executable Python code

**Requirements**:
- H2 heading "Practical Example: [specific scenario]"
- 10-30 lines of Python code (type hints mandatory)
- Use ROS2 rclpy library
- Include inline comments explaining key lines
- Code should be **conceptual** (not production-ready, can have `pass` for unimplemented parts)
- Follow by 2-3 paragraphs explaining the code

**Example** (Lesson 1 - Camera Systems):
```markdown
## Practical Example: Subscribing to Camera Images

Here's a simple ROS2 node that subscribes to a camera's image topic and logs the image dimensions:

​```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    """Subscribes to camera images and processes metadata."""

    def __init__(self) -> None:
        super().__init__('camera_subscriber')
        # Subscribe to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS queue depth
        )

    def image_callback(self, msg: Image) -> None:
        """Process incoming camera image."""
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}, encoding={msg.encoding}'
        )
        # Image processing would go here (e.g., OpenCV operations)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
​```

This node demonstrates the basic pattern for camera data processing in ROS2. The `image_callback` receives `sensor_msgs/Image` messages containing the raw pixel data (`msg.data`), dimensions (`width`, `height`), and pixel format (`encoding` like "rgb8" or "bgr8").

In a real humanoid robot, this callback would pass the image to a computer vision pipeline (object detection, face recognition, etc.) using libraries like OpenCV or PyTorch.
```

**Validation**:
- [ ] H2 heading with "Practical Example:" prefix
- [ ] Code block with Python syntax highlighting
- [ ] Type hints on all function signatures
- [ ] 2-3 paragraphs explaining code after the block
- [ ] Code references correct ROS2 message types

---

### Section 6: Summary

**Purpose**: Reinforce key takeaways

**Requirements**:
- H2 heading "Summary"
- Bulleted list (3-5 items)
- Each bullet: 1 sentence capturing main concept
- Concise (total 100-200 words)

**Example** (Lesson 4 - Sensor Fusion):
```markdown
## Summary

- **Sensor fusion** combines data from multiple sensors (cameras, LiDAR, IMU) to create more accurate and robust perception than any single sensor alone.
- **Kalman filters** optimally estimate robot state by balancing noisy sensor measurements with motion predictions, weighted by uncertainty.
- **Complementary filters** provide a simpler fusion approach, combining high-frequency sensors (gyroscopes) with low-frequency sensors (accelerometers) using frequency-based filtering.
- **Visual-Inertial Odometry (VIO)** fuses camera and IMU data for accurate indoor localization where GPS is unavailable, critical for humanoid navigation.
- ROS2's **robot_localization** package automates multi-sensor fusion, handling timestamp alignment and covariance-weighted integration.
```

**Validation**:
- [ ] H2 heading "Summary"
- [ ] 3-5 bulleted items
- [ ] Each bullet is one complete sentence
- [ ] Covers main concepts from all sections

---

### Section 7: Next Steps

**Purpose**: Guide students to subsequent content

**Requirements**:
- H2 heading "Next Steps"
- 1-2 sentences previewing the next lesson
- Link to next lesson using Docusaurus ID format (e.g., `./02-depth-sensing` not `./02-depth-sensing.md`)
- For Lesson 4 (final lesson), link to capstone project

**Example** (Lesson 2):
```markdown
## Next Steps

Now that you understand how humanoid robots measure depth using LiDAR and depth cameras, the next lesson explores how robots sense their own body motion and orientation using Inertial Measurement Units (IMUs).

Continue to [Lesson 3: IMU and Proprioception](./03-imu-proprioception)
```

**Example** (Lesson 4):
```markdown
## Next Steps

You've learned about individual sensor types (cameras, depth sensors, IMUs) and how to combine them through sensor fusion. Now it's time to apply this knowledge in a comprehensive design exercise.

Continue to the [Capstone Project: Multi-Sensor Perception System](./05-capstone-project)
```

**Validation**:
- [ ] H2 heading "Next Steps"
- [ ] Preview text for next content
- [ ] Link uses Docusaurus ID format (no `.md` extension)

---

## Code Example Standards

### Python Style Requirements

1. **Type Hints**: Mandatory on all function signatures
   ```python
   def image_callback(self, msg: Image) -> None:
   ```

2. **Docstrings**: Class-level docstring required, function docstrings optional but recommended
   ```python
   class CameraSubscriber(Node):
       """Subscribes to camera images and processes metadata."""
   ```

3. **Imports**: Group by standard library, third-party, ROS2
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   ```

4. **Length**: 10-30 lines (conceptual snippets, not full applications)

5. **Comments**: Explain non-obvious logic, especially ROS2-specific patterns

### ROS2 Message Type References

- Use fully qualified names: `sensor_msgs/Image` (not `Image` alone)
- Reference official ROS2 Humble message definitions
- Show message structure in comments when introducing new type:
  ```python
  # sensor_msgs/Imu contains:
  #   - orientation: geometry_msgs/Quaternion
  #   - angular_velocity: geometry_msgs/Vector3 (rad/s)
  #   - linear_acceleration: geometry_msgs/Vector3 (m/s²)
  ```

---

## Validation Checklist (Per Lesson)

### Structure

- [ ] All 7 sections present in order
- [ ] H2 headings for main sections, H3 for subsections
- [ ] Section lengths within specified ranges

### Content Quality

- [ ] No placeholder text (e.g., [TODO], [FILL IN])
- [ ] All technical terms defined on first use
- [ ] At least one humanoid robot example (Atlas, Optimus, Digit)
- [ ] ROS2 concepts referenced correctly

### Callouts

- [ ] 1-2 AI Colearning Prompts with specific questions
- [ ] 1-2 Expert Insights with pitfalls or advanced tips
- [ ] 1-2 Practice Exercises with clear tasks

### Code

- [ ] Python code follows style requirements (type hints, docstrings)
- [ ] Code is 10-30 lines (conceptual, not production)
- [ ] ROS2 message types referenced correctly
- [ ] Code explanation paragraphs follow code block

### Navigation

- [ ] Summary captures 3-5 key takeaways
- [ ] Next Steps previews following content
- [ ] Links use Docusaurus ID format (no file extensions)

---

## Examples by Lesson

**Lesson 1** (Camera Systems):
- What: Definition of cameras in robotics
- Why: Visual perception enables object recognition, navigation
- Key Principles: Monocular vs stereo vs RGB-D comparison
- Example: Subscribing to sensor_msgs/Image topic
- Summary: Camera types, ROS2 messages, humanoid use cases

**Lesson 2** (Depth Sensing):
- What: Depth sensors measure 3D distance
- Why: Manipulation and navigation require depth perception
- Key Principles: LiDAR, structured light, ToF comparison
- Example: Subscribing to sensor_msgs/PointCloud2 topic
- Summary: Depth sensor types, trade-offs, point cloud processing

**Lesson 3** (IMU & Proprioception):
- What: IMU measures body motion and orientation
- Why: Balance control and self-awareness for bipedal robots
- Key Principles: Accelerometer, gyroscope, magnetometer, drift
- Example: Subscribing to sensor_msgs/Imu topic
- Summary: IMU components, proprioception, balance applications

**Lesson 4** (Sensor Fusion):
- What: Combining multiple sensors for robust perception
- Why: Single sensors have limitations, fusion provides redundancy
- Key Principles: Kalman filter, complementary filter, VIO
- Example: Conceptual multi-sensor node architecture
- Summary: Fusion strategies, ROS2 robot_localization, timestamp alignment

---

## Summary

This contract ensures:
- **Consistency**: All Module 2 lessons follow the same 7-section structure
- **RAG-Readiness**: Clear semantic sections (H2/H3 hierarchy) for chunking
- **Educational Quality**: Callouts, examples, and summaries support learning
- **Technical Accuracy**: ROS2 code follows best practices, references correct message types
- **Student Guidance**: Next Steps links create clear learning progression

**Compliance**: All lesson content MUST pass this contract validation before Technical Review Agent step.
