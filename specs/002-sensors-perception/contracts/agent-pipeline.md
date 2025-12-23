# Agent Pipeline Contract

**Feature**: 002-sensors-perception
**Purpose**: Define input/output schemas and acceptance criteria for the 9-step content creation pipeline

## Overview

Module 2 follows the same 9-agent pipeline established in Module 1 (constitution v1.0, section III). This contract specifies what each agent receives as input, what it must produce as output, how to validate success, and how to handle failures. All agents are **stateless** (no memory between invocations) and **reusable** (can be applied to any sensor perception lesson).

---

## Pipeline Flow

```
User-Approved Spec (spec.md)
  ↓
┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: CONTENT CREATION (Agents 1-4)                      │
├─────────────────────────────────────────────────────────────┤
│ [1] Outline Agent → structured subsections                   │
│ [2] Chapter Content Agent → full MDX content                 │
│ [3] Case Study Generator → real-world robotics examples      │
│ [4] Code Example Agent → ROS2/Python/sensor_msgs snippets   │
└─────────────────────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2: VALIDATION (Agent 5)                               │
├─────────────────────────────────────────────────────────────┤
│ [5] Technical Reviewer Agent → validates accuracy           │
│     (ROS2 concepts, sensor physics, message structures)      │
└─────────────────────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────────────────────┐
│ PHASE 3: FORMATTING (Agents 6-8)                            │
├─────────────────────────────────────────────────────────────┤
│ [6] Structure & Style Agent → enforces 7-section pattern    │
│ [7] Frontmatter Agent → generates 13-field YAML metadata    │
│ [8] Docusaurus Integration Agent → converts to final MDX    │
└─────────────────────────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────────────────────────┐
│ PHASE 4: APPROVAL (Agent 9 = Human)                         │
├─────────────────────────────────────────────────────────────┤
│ [9] Review Checkpoint → User validates & approves           │
└─────────────────────────────────────────────────────────────┘
  ↓
Commit to feature branch
```

---

## Agent 1: Outline Agent

### Input Schema

```yaml
input:
  topic: string                      # Lesson topic (e.g., "Camera Systems and Computer Vision")
  learning_goals: array<string>      # 3-5 learning goals from spec (e.g., ["differentiate camera types", "understand Image messages"])
  target_audience: string            # "CS students with Python + Module 1 (ROS2) knowledge"
  prerequisites: array<string>       # Topics students must know (e.g., ["ROS2 topics", "pub/sub pattern"])
  module_context: string             # Module 2 focus (sensors and perception for humanoid robots)
```

### Output Schema

```yaml
output:
  type: "markdown"
  structure:
    - section_name: string           # H2 heading (e.g., "What Is a Camera in Robotics?")
      subsections: array<string>     # H3 subheadings (e.g., ["Monocular Cameras", "Stereo Cameras"])
      content_guidance: string       # What to cover in this section
      estimated_words: integer       # Target word count (200-500)
```

**Example Output** (Lesson 1 - Camera Systems):
```markdown
## Outline: Camera Systems and Computer Vision

### Section 1: What Is a Camera in Robotics?
- Subsections: [Definition, Role in Humanoid Robots, Key Terminology]
- Content Guidance: Define camera as visual sensor, explain light-to-digital conversion, introduce terminology (pixel, resolution, FOV)
- Estimated Words: 300

### Section 2: Why Camera Systems Matter for Physical AI
- Subsections: [Visual Perception Capabilities, Real-World Examples, Limitations Without Vision]
- Content Guidance: Explain object recognition, navigation, human interaction use cases. Include Tesla Optimus or Boston Dynamics example.
- Estimated Words: 400

### Section 3: Key Principles
- Subsections: [Monocular Cameras, Stereo Cameras, RGB-D Cameras, Trade-Off Analysis]
- Content Guidance: Compare 3 camera types with table (depth info, range, use cases). Explain triangulation for stereo.
- Estimated Words: 600

### Section 4: Practical Example - ROS2 Camera Subscriber
- Subsections: [sensor_msgs/Image Structure, Code Example, Explanation]
- Content Guidance: Show Python node subscribing to /camera/image_raw, explain message fields, demonstrate logging image dimensions.
- Estimated Words: 400

### Section 5: RViz Visualization
- Subsections: [Visualizing Camera Images, Configuration Steps]
- Content Guidance: Guide students through RViz Image display type, selecting topic, inspecting camera feed.
- Estimated Words: 300

### Section 6: Summary
- Content Guidance: 3-5 bullet points capturing camera types, ROS2 messages, humanoid applications
- Estimated Words: 150

### Section 7: Next Steps
- Content Guidance: Preview Lesson 2 (depth sensing), link to ./02-depth-sensing
- Estimated Words: 100
```

### Acceptance Criteria

- [ ] Outline follows 7-section structure from lesson-structure.md contract
- [ ] All learning goals from input covered across sections
- [ ] H2 and H3 headings follow naming conventions ("What Is...", "Why... Matters", "Key Principles", etc.)
- [ ] Estimated word counts sum to 2000-3000 words total
- [ ] No placeholder text (no [TODO], [TBD])

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Missing learning goal | Input unclear or goal omitted | Add clarification: "Ensure all learning goals in input are addressed" |
| Wrong section structure | Agent deviates from 7-section template | Retry with explicit constraint: "MUST follow exactly 7 sections from contract" |
| Too brief | Word count estimates too low | Adjust prompt: "Aim for 2000-3000 words total, expand Key Principles section" |

---

## Agent 2: Chapter Content Agent

### Input Schema

```yaml
input:
  outline: object                    # Output from Outline Agent
  sensor_type: string                # Specific sensor (e.g., "camera", "lidar", "imu")
  ros2_messages: array<string>       # Relevant ROS2 message types (e.g., ["sensor_msgs/Image", "sensor_msgs/CameraInfo"])
  humanoid_examples: array<string>   # Case study robots (e.g., ["Tesla Optimus", "Boston Dynamics Atlas"])
  callout_requirements: object       # {ai_colearning_prompts: 2, expert_insights: 1, practice_exercises: 1}
```

### Output Schema

```yaml
output:
  type: "markdown"
  content: string                    # Full lesson markdown (2000-3000 words)
  sections:
    - What Is [Concept]
    - Why [Concept] Matters for Physical AI
    - Key Principles (with H3 subsections)
    - Practical Example (with code block)
    - Summary (bulleted list)
    - Next Steps (with link)
  callouts_embedded: array<object>   # [{type: "ai_colearning", location: "section3"}, ...]
```

### Acceptance Criteria

- [ ] Content matches outline structure (all sections present)
- [ ] Length 2000-3000 words
- [ ] All callouts embedded (💬 AI Colearning, 🎓 Expert Insight, 🤝 Practice Exercise)
- [ ] ROS2 message types referenced correctly (sensor_msgs/Image not sensor_msg/Image)
- [ ] At least one humanoid robot example included with specific scenario
- [ ] No broken internal links
- [ ] Code blocks use Python syntax highlighting (```python)

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Missing callouts | Agent forgot to embed prompts | Add checklist: "Before finishing, verify 2 AI prompts, 1 expert insight, 1 practice exercise present" |
| Incorrect ROS2 syntax | Agent uses wrong package name | Provide reference: "Use sensor_msgs (plural) package, check message spelling against ROS2 Humble docs" |
| Too technical | Content assumes advanced knowledge | Simplify prompt: "Target CS students with basic Python, avoid advanced linear algebra" |

---

## Agent 3: Case Study Generator

### Input Schema

```yaml
input:
  lesson_topic: string               # (e.g., "Sensor Fusion")
  robot_examples: array<string>      # Robots to feature (e.g., ["Boston Dynamics Atlas", "Agility Digit"])
  focus_area: string                 # What to highlight (e.g., "visual-inertial odometry for indoor navigation")
  desired_length: integer            # 200-400 words
```

### Output Schema

```yaml
output:
  type: "markdown"
  case_study:
    title: string                    # (e.g., "Case Study: Visual-Inertial Odometry in Agility Digit")
    scenario: string                 # Real-world task description
    sensors_used: array<string>      # (e.g., ["stereo cameras", "torso IMU"])
    fusion_strategy: string          # (e.g., "Extended Kalman Filter combining visual features and IMU")
    outcome: string                  # Result or benefit
    source: string                   # Reference (e.g., "Agility Robotics technical documentation")
```

**Example Output** (Lesson 4 - Sensor Fusion):
```markdown
### Case Study: Visual-Inertial Odometry in Boston Dynamics Atlas

**Scenario**: Atlas navigates through a cluttered warehouse interior where GPS is unavailable and lighting varies dramatically (bright windows, dark corners). The robot must maintain accurate localization to plan collision-free paths.

**Sensors Used**:
- Head-mounted stereo cameras (640×480, 30 Hz)
- Torso-mounted 9-DOF IMU (200 Hz)

**Fusion Strategy**: Atlas employs **visual-inertial odometry (VIO)** using an Extended Kalman Filter. The stereo cameras track visual features (corners, edges) between frames to estimate motion, while the IMU provides high-frequency orientation and acceleration data. The Kalman filter fuses these measurements, weighting camera data higher when features are abundant (well-lit areas) and trusting IMU more during fast motions or low-light conditions.

**Outcome**: Atlas achieves <2% positional drift over 100-meter traversals, enabling precise navigation without external infrastructure. The fusion approach overcomes camera failures (motion blur, darkness) and IMU drift (gyroscope integration errors) that would cripple single-sensor systems.

**Source**: Boston Dynamics technical presentations (2023), IEEE Robotics & Automation Letters
```

### Acceptance Criteria

- [ ] Includes robot name, scenario, sensors, fusion strategy, outcome
- [ ] Length 200-400 words
- [ ] Grounded in real robotics (no fictional scenarios)
- [ ] Source cited (technical paper, company documentation, conference talk)
- [ ] Connects to lesson topic (reinforces key concepts)

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Fictional details | Agent invents unrealistic specs | Constrain: "Only use publicly documented sensor configurations from official sources" |
| Too vague | Lacks technical specifics | Request: "Include sensor update rates, fusion algorithm name, quantitative outcome (e.g., <2% drift)" |
| Off-topic | Case study doesn't illustrate lesson concept | Refocus: "Case study MUST demonstrate [specific fusion strategy] from lesson" |

---

## Agent 4: Code Example Agent

### Input Schema

```yaml
input:
  lesson_topic: string               # (e.g., "Subscribing to IMU Data")
  ros2_message_type: string          # (e.g., "sensor_msgs/Imu")
  code_purpose: string               # What code should demonstrate (e.g., "log angular velocity from gyroscope")
  complexity_level: enum             # beginner | intermediate
  length_constraint: integer         # 10-30 lines
```

### Output Schema

```yaml
output:
  type: "python"
  code_block: string                 # Executable Python code with type hints
  explanation: string                # 2-3 paragraphs explaining code (200-300 words)
  key_points: array<string>          # 3-5 bullet points highlighting important concepts
```

**Example Output** (Lesson 3 - IMU):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    """Subscribes to IMU data and logs angular velocity."""

    def __init__(self) -> None:
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg: Imu) -> None:
        """Process incoming IMU measurements."""
        angular_vel = msg.angular_velocity
        self.get_logger().info(
            f'Gyroscope: x={angular_vel.x:.2f}, '
            f'y={angular_vel.y:.2f}, '
            f'z={angular_vel.z:.2f} rad/s'
        )
        # In a real robot, this data feeds into balance control
```

**Explanation**:
This node demonstrates the basic pattern for accessing gyroscope data from a humanoid robot's IMU. The `sensor_msgs/Imu` message contains three key fields: `orientation` (estimated pose), `angular_velocity` (gyroscope readings in rad/s), and `linear_acceleration` (accelerometer readings in m/s²).

In this example, we extract the `angular_velocity` vector, which tells us how fast the robot is rotating around each axis (x=roll rate, y=pitch rate, z=yaw rate). For a humanoid maintaining balance, rapid changes in angular velocity (e.g., z > 1.0 rad/s) indicate the robot is tipping and needs corrective action.

Real implementations would pass this data to a balance controller (e.g., Linear Quadratic Regulator) that commands ankle/hip torques to stabilize the robot. The high update rate of IMUs (100-1000 Hz) enables fast feedback loops essential for bipedal locomotion.

**Key Points**:
- `sensor_msgs/Imu` contains orientation, angular_velocity, and linear_acceleration
- Gyroscope measures rotation rate (rad/s) around x, y, z axes
- High-frequency IMU data (100+ Hz) enables real-time balance control
- Angular velocity feeds into control loops (not used in isolation)

### Acceptance Criteria

- [ ] Code is 10-30 lines (conceptual, not production-ready)
- [ ] Python 3.11+ with type hints on all functions
- [ ] Uses rclpy library correctly (Node subclass, create_subscription pattern)
- [ ] ROS2 message type imported and used correctly
- [ ] Explanation is 200-300 words
- [ ] Key points (3-5 bullets) highlight important concepts
- [ ] Code can be understood without running it (conceptual demonstration)

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Missing type hints | Agent omits type annotations | Enforce: "MUST include type hints on __init__ and callback functions" |
| Too complex | Code includes unnecessary advanced features | Simplify: "Remove extra logic, focus only on subscribing and logging message fields" |
| Wrong message import | Uses incorrect package or message name | Correct: "Import from sensor_msgs.msg, not std_msgs or sensor_msg" |

---

## Agent 5: Technical Reviewer Agent

### Input Schema

```yaml
input:
  lesson_markdown: string            # Full lesson content from Agent 2
  code_examples: array<string>       # All code blocks from lesson
  technical_domain: string           # (e.g., "ROS2 sensor_msgs", "IMU physics", "LiDAR principles")
  validation_checklist: array<string> # Specific checks (e.g., ["sensor_msgs/Image fields correct", "gyroscope drift explained accurately"])
```

### Output Schema

```yaml
output:
  type: "validation_report"
  status: enum                       # pass | fail | needs_revision
  technical_errors: array<object>    # [{error: "Incorrect LaserScan field name", location: "line 45", correction: "Use 'ranges' not 'range'"}]
  accuracy_score: float              # 0.0-1.0 (1.0 = fully accurate)
  approval: boolean                  # true if status=pass, false otherwise
  suggested_corrections: string      # Markdown diff or prose corrections
```

### Validation Checklist (Module 2)

**ROS2 Message Accuracy**:
- [ ] sensor_msgs/Image fields: header, height, width, encoding, is_bigendian, step, data
- [ ] sensor_msgs/PointCloud2 fields: header, height, width, fields, is_bigendian, point_step, row_step, data, is_dense
- [ ] sensor_msgs/Imu fields: header, orientation, orientation_covariance, angular_velocity, angular_velocity_covariance, linear_acceleration, linear_acceleration_covariance
- [ ] sensor_msgs/LaserScan fields: header, angle_min, angle_max, angle_increment, range_min, range_max, ranges, intensities

**Sensor Physics Accuracy**:
- [ ] Camera types (monocular, stereo, RGB-D) described correctly
- [ ] LiDAR time-of-flight principle accurate
- [ ] Accelerometer measures linear acceleration (not velocity)
- [ ] Gyroscope measures angular velocity (not orientation directly)
- [ ] Drift explanation accurate (gyroscope integration error accumulation)

**ROS2 Concepts**:
- [ ] Topic names follow conventions (/camera/image_raw not /camera_image)
- [ ] QoS depth values reasonable (10 is standard for sensor topics)
- [ ] rclpy API usage correct (create_subscription syntax, callback signature)

**Humanoid Robotics Accuracy**:
- [ ] Robot examples (Atlas, Optimus, Digit) use documented capabilities
- [ ] Balance control explanations align with control theory (COM, support polygon)
- [ ] Sensor fusion strategies (VIO, Kalman filter) described accurately

### Acceptance Criteria

- [ ] All technical errors identified with specific corrections
- [ ] Accuracy score >= 0.95 (95% correct)
- [ ] No critical errors (wrong message field names, incorrect physics)
- [ ] Suggested corrections actionable (specific line numbers or sections)

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Missed errors | Agent doesn't catch incorrect message field | Strengthen checklist: "Validate ALL message fields against official ROS2 Humble sensor_msgs definitions" |
| False positives | Agent flags correct content as error | Provide reference: "Check correction against ROS2 official docs before flagging" |
| Vague corrections | "Fix sensor description" without specifics | Require format: "Error: [what's wrong], Correction: [what to write instead], Location: [line/section]" |

---

## Agent 6: Structure & Style Agent

### Input Schema

```yaml
input:
  lesson_markdown: string            # Content from Agent 2 (after technical review)
  structure_contract: string         # lesson-structure.md contract
  validation_rules: array<string>    # ["7 sections required", "H2/H3 hierarchy", "callouts embedded"]
```

### Output Schema

```yaml
output:
  type: "formatted_markdown"
  content: string                    # Reformatted lesson with correct structure
  violations_fixed: array<object>    # [{violation: "Missing H2 Summary section", fix: "Added Summary section with 4 bullets"}]
  compliance_score: float            # 0.0-1.0 (1.0 = fully compliant)
```

### Validation Rules

- [ ] Exactly 7 main sections (H2 headings)
- [ ] Section order correct (What Is → Why Matters → Key Principles → Example → Summary → Next Steps)
- [ ] H3 subheadings under Key Principles (3-5 subheadings)
- [ ] Callouts embedded (💬 2, 🎓 1, 🤝 1 minimum)
- [ ] Code block has Python syntax highlighting
- [ ] Summary is bulleted list (3-5 items)
- [ ] Next Steps has link to next lesson (Docusaurus ID format)

### Acceptance Criteria

- [ ] All structure violations corrected
- [ ] Compliance score >= 0.98
- [ ] Heading hierarchy valid (no H4 without H3, no H3 without H2)
- [ ] Callouts use correct emoji and blockquote format

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Section missing | Agent skipped Summary or Next Steps | Auto-generate: "If Summary missing, extract 3-5 key points from content and create bulleted list" |
| Wrong heading level | H3 used for main sections instead of H2 | Enforce: "Main sections MUST be H2 (##), subsections H3 (###)" |
| Callout format wrong | Missing emoji or blockquote | Auto-fix: "Ensure format is `### 💬 AI Colearning Prompt\n\n> [content]`" |

---

## Agent 7: Frontmatter Agent

### Input Schema

```yaml
input:
  lesson_content: string             # Full lesson markdown from Agent 6
  lesson_number: integer             # 1-6 (01-04 lessons, 05 capstone, 06 quiz)
  module_number: integer             # 2 (for Module 2)
  learning_objectives: array<string> # Extracted from spec.md
  frontmatter_schema: object         # frontmatter-schema.yaml contract
```

### Output Schema

```yaml
output:
  type: "yaml"
  frontmatter:
    title: string
    sidebar_position: integer
    skills: array<object>
    learning_objectives: array<object>
    cognitive_load: object
    differentiation: object
    tags: array<string>
    generated_by: string
    created: string
    last_modified: string
```

### Acceptance Criteria

- [ ] All 13 required fields present
- [ ] No additional custom fields
- [ ] sidebar_position matches lesson_number
- [ ] Skills (1-3) with all 6 sub-fields
- [ ] Learning objectives (3-5) with all 4 sub-fields
- [ ] Cognitive load has new_concepts (3-8) and assessment
- [ ] Differentiation has extension and remedial (both non-empty)
- [ ] Tags (3-5) all lowercase, from controlled vocabulary
- [ ] generated_by = "agent"
- [ ] created/last_modified in YYYY-MM-DD format

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Missing field | Agent omits cognitive_load or differentiation | Validate: "Check all 13 fields present before returning" |
| Wrong enum value | bloom_level = "comprehension" (not in taxonomy) | Enforce: "bloom_level MUST be one of: remember, understand, apply, analyze, evaluate, create" |
| Tag not in vocabulary | Uses "cameras" instead of "camera" | Validate: "Check tags against Module 2 controlled vocabulary in frontmatter-schema.yaml" |

---

## Agent 8: Docusaurus Integration Agent

### Input Schema

```yaml
input:
  lesson_markdown: string            # From Agent 6 (structured content)
  frontmatter_yaml: object           # From Agent 7
  module_directory: string           # "book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/"
  lesson_filename: string            # (e.g., "01-camera-systems.md")
  image_paths: array<string>         # SVG diagrams to reference
```

### Output Schema

```yaml
output:
  type: "mdx"
  lesson_file:
    path: string                     # Full file path
    content: string                  # Frontmatter + markdown content
  summary_file:
    path: string                     # (e.g., "01-camera-systems.summary.md")
    content: string                  # Extracted summary section + frontmatter
  validation: object
    docusaurus_build: boolean        # True if build succeeds
    broken_links: array<string>      # Any broken internal links
```

### Acceptance Criteria

- [ ] Lesson file written to correct path
- [ ] Frontmatter enclosed in `---` delimiters
- [ ] Summary file created with same frontmatter + Summary section content
- [ ] Image paths use correct format (`![alt](/img/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/diagram.svg)`)
- [ ] Internal links use Docusaurus ID format (`./02-depth-sensing` not `./02-depth-sensing.md`)
- [ ] Docusaurus build succeeds (`npm run build` returns 0)

### Failure Modes

| Failure | Cause | Retry Strategy |
|---------|-------|----------------|
| Build fails | Broken link or invalid MDX syntax | Parse error message, fix broken link (e.g., `./lesson.md` → `./lesson`) |
| Summary file missing | Agent forgot to create paired file | Auto-generate: "Extract Summary section, prepend frontmatter, write to `NN-lesson-name.summary.md`" |
| Image not found | Referenced SVG doesn't exist | Create placeholder: "Generate placeholder SVG at specified path with 'Placeholder for [diagram name]' text" |

---

## Agent 9: Review Checkpoint (Human)

### Input

- Lesson file preview (Docusaurus rendered page)
- Summary file preview
- Frontmatter metadata display
- Technical Reviewer report
- Structure & Style compliance score

### Output

- **Approval** (proceed to commit) OR
- **Revision Request** (return to specific agent with feedback)

### Acceptance Criteria

- [ ] Content matches spec learning objectives
- [ ] Technical accuracy validated
- [ ] Structure follows 7-section pattern
- [ ] Code examples clear and correct
- [ ] Frontmatter complete and accurate
- [ ] Renders correctly on Docusaurus page

---

## Summary

This agent pipeline contract defines:
- **9 agents** with clear input/output schemas
- **Acceptance criteria** for each agent's output
- **Failure modes** and retry strategies
- **Validation checklists** for technical accuracy and structure
- **Stateless design** enabling agent reuse across lessons

**Compliance**: All Module 2 content MUST pass through this 9-step pipeline before merging to main branch.
