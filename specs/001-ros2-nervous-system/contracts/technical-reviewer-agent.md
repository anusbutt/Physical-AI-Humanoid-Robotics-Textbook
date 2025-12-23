# Agent Contract: Technical Reviewer Agent

**Agent Type**: Validation Agent
**Purpose**: Validate technical accuracy of ROS2, rclpy, and URDF content
**Status**: Critical - Implement for all lessons (Constitution Principle V)
**Priority**: HIGH (NON-NEGOTIABLE quality gate)

## Input Schema

```yaml
lesson_content:
  type: string
  required: true
  description: "Full markdown content of lesson (including frontmatter)"
  format: "markdown"

focus_areas:
  type: array<string>
  required: true
  description: "Which technical domains to validate"
  options:
    - "ros2_concepts"      # Nodes, topics, services, parameters, graph
    - "rclpy_api"          # Python rclpy library usage
    - "urdf_syntax"        # XML structure, links, joints
    - "gazebo_concepts"    # Physics simulation (if applicable)
    - "pedagogy"           # Explanations match student level
  example: ["ros2_concepts", "rclpy_api", "pedagogy"]

ros2_version:
  type: string
  required: false
  default: "humble"
  description: "Which ROS2 distribution to validate against"

student_level:
  type: string
  required: false
  default: "beginner"
  enum: ["beginner", "intermediate", "advanced"]
  description: "Expected student background (affects pedagogy validation)"
```

## Output Schema

```yaml
type: json
structure:
  validation_result:
    type: string
    enum: ["PASS", "FAIL", "NEEDS_REVISION"]

  issues:
    type: array<object>
    properties:
      severity:
        type: string
        enum: ["critical", "major", "minor"]
      category:
        type: string
        enum: ["ros2_accuracy", "rclpy_api", "urdf_syntax", "pedagogical", "terminology"]
      location:
        type: string
        description: "Section or line reference where issue found"
      issue:
        type: string
        description: "What is incorrect or unclear"
      suggestion:
        type: string
        description: "How to fix the issue"
      reference:
        type: string
        description: "Link to official docs or authoritative source"

  validation_checks:
    type: object
    properties:
      ros2_concepts_accurate:
        type: boolean
      rclpy_api_correct:
        type: boolean
      urdf_syntax_valid:
        type: boolean
      explanations_appropriate:
        type: boolean
      code_examples_valid:
        type: boolean
      terminology_consistent:
        type: boolean

  summary:
    type: string
    description: "Overall assessment and recommendation"

example:
  validation_result: "NEEDS_REVISION"
  issues:
    - severity: "critical"
      category: "ros2_accuracy"
      location: "Section: What Is ROS2?"
      issue: "States ROS2 has a master node, but ROS2 is master-less (unlike ROS1)"
      suggestion: "Replace with: 'ROS2 uses a distributed discovery system (DDS) with no central master node, unlike ROS1.'"
      reference: "https://docs.ros.org/en/humble/Concepts/Basic/About-Different-Middleware-Vendors.html"
  validation_checks:
    ros2_concepts_accurate: false
    rclpy_api_correct: true
    explanations_appropriate: true
  summary: "1 critical issue found. Fix master node misconception before proceeding."
```

## Acceptance Criteria

- [ ] All technical claims validated against official ROS2/rclpy/URDF documentation
- [ ] Code examples use correct API syntax for specified ROS2 version
- [ ] Terminology is consistent with official ROS2 glossary
- [ ] Explanations are accurate and appropriate for student level
- [ ] No misleading analogies or outdated information (ROS1 vs ROS2 confusion)
- [ ] All issues include specific location references
- [ ] All critical/major issues include actionable suggestions
- [ ] References link to authoritative sources (docs.ros.org, official GitHub)
- [ ] Validation result is clear: PASS, FAIL, or NEEDS_REVISION

## Failure Modes

| Failure Mode | Trigger | Recovery Strategy |
|--------------|---------|------------------|
| **False positive** | Agent flags correct content as incorrect | Human review override; update agent knowledge base |
| **False negative** | Agent misses actual technical error | Post-deployment correction; update agent validation rules |
| **Vague feedback** | Issue description unclear ("something wrong with code") | Require specific location + explanation in output schema |
| **Outdated reference** | Agent validates against old ROS2 version | Enforce `ros2_version` input parameter; validate against correct docs |
| **Pedagogical mismatch** | Content technically correct but too complex for beginners | Check `student_level` input; flag if explanation assumes advanced knowledge |

## Validation Checklist (Internal to Agent)

**ROS2 Concepts**:
- [ ] Nodes described correctly (independent processes, reusable)
- [ ] Topics explained as pub/sub (asynchronous, many-to-many)
- [ ] Services explained as request/response (synchronous, one-to-one)
- [ ] Parameters described correctly (configuration, dynamic reconfigure)
- [ ] ROS2 graph concept accurate (nodes + topics + services visualization)
- [ ] DDS layer mentioned where relevant (no master node)
- [ ] QoS (Quality of Service) not oversimplified (if mentioned)

**rclpy API**:
- [ ] `rclpy.init()` and `rclpy.shutdown()` usage correct
- [ ] Node class inherits from `rclpy.node.Node`
- [ ] Publisher created with `create_publisher(msg_type, topic, qos)`
- [ ] Subscriber created with `create_subscription(msg_type, topic, callback, qos)`
- [ ] Timer created with `create_timer(period, callback)`
- [ ] Message types imported correctly (e.g., `from std_msgs.msg import String`)
- [ ] Method signatures match ROS2 Humble API

**URDF Syntax**:
- [ ] XML structure valid (`<robot>`, `<link>`, `<joint>` tags)
- [ ] Links have required elements (`<visual>`, `<collision>`, `<inertial>` where appropriate)
- [ ] Joints specify type (revolute, prismatic, fixed, continuous)
- [ ] Parent-child relationships valid (no circular dependencies)
- [ ] Units consistent (meters, radians as per URDF spec)
- [ ] Attribute names correct (`name`, `type`, `xyz`, `rpy`)

**Pedagogy**:
- [ ] Explanations assume only stated prior knowledge (Python, basic networking)
- [ ] Technical jargon defined on first use
- [ ] Analogies are accurate and helpful (not misleading)
- [ ] Code examples match claimed conceptual focus (10-30 lines, not full apps)
- [ ] Callouts (💬🎓🤝) are appropriate for student level

## Usage Example

**Input**:
```yaml
lesson_content: |
  # ROS2 Fundamentals

  ## What Is ROS2?
  ROS2 is the successor to ROS1, with a central master node that coordinates all communication...

  ## Practical Example
  ```python
  import rclpy
  from rclpy.node import Node

  class SimpleNode(Node):
      def __init__(self):
          Node.__init__(self, 'simple_node')  # Incorrect: should use super()
  ```

focus_areas: ["ros2_concepts", "rclpy_api"]
ros2_version: "humble"
student_level: "beginner"
```

**Expected Output**:
```json
{
  "validation_result": "NEEDS_REVISION",
  "issues": [
    {
      "severity": "critical",
      "category": "ros2_accuracy",
      "location": "Section: What Is ROS2?, Paragraph 1",
      "issue": "States 'central master node' exists in ROS2, but ROS2 is master-less (uses DDS discovery)",
      "suggestion": "Replace with: 'ROS2 uses a distributed discovery mechanism via DDS, eliminating the single-point-of-failure master node from ROS1.'",
      "reference": "https://docs.ros.org/en/humble/Concepts/Basic/About-DDS-Implementations.html"
    },
    {
      "severity": "major",
      "category": "rclpy_api",
      "location": "Practical Example, line 6",
      "issue": "Uses Node.__init__() directly instead of super().__init__()",
      "suggestion": "Change to: super().__init__('simple_node')",
      "reference": "https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html"
    }
  ],
  "validation_checks": {
    "ros2_concepts_accurate": false,
    "rclpy_api_correct": false,
    "explanations_appropriate": true
  },
  "summary": "2 issues found (1 critical, 1 major). Fix master node misconception and rclpy inheritance pattern before proceeding."
}
```

## Implementation Notes

- Implement immediately for all lessons (not iterative like other agents)
- Agent must have access to ROS2 Humble documentation
- Run after Content Agent, before Structure & Style Agent
- Critical/major issues block progression to next pipeline step
- Minor issues logged but don't block (user decides whether to fix)
- Agent retries content automatically if author provides revised version
- Human review available for disputes (false positive override)
