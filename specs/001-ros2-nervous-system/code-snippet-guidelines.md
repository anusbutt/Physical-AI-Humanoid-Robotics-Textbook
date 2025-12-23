# Code Snippet Guidelines - Module 1: ROS2 Nervous System

**Purpose**: Standards for creating Python code examples in Module 1 lessons
**Based on**: research.md decision "Conceptual Code Snippets (10-30 lines)"
**Target**: CS students with Python knowledge
**Usage**: Follow these guidelines for all code examples in Lessons 3-4

---

## Core Principles

### 1. Conceptual Focus, Not Executable Tutorials

**Goal**: Students understand the pattern/structure, not debug environment setup

**What This Means**:
- Code demonstrates the IDEA, not the full implementation
- Omit boilerplate (error handling, imports beyond essentials, full lifecycle)
- Focus on the 20% of code that teaches 80% of the concept
- Students can read and understand without running locally

**Example - DO THIS**:
```python
class SimplePublisher(Node):
    """Publishes robot status messages."""

    def __init__(self) -> None:
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self) -> None:
        msg = String()
        msg.data = 'Robot operational'
        self.publisher.publish(msg)
```

**Example - DON'T DO THIS** (too much boilerplate):
```python
def main(args=None):
    try:
        rclpy.init(args=args)
        node = SimplePublisher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Code Requirements

### Length: 10-30 Lines

**Rationale**: Fits on one screen, doesn't overwhelm, focuses attention

**How to Achieve**:
- One concept per snippet (publisher OR subscriber, not both)
- Omit `main()`, `if __name__`, error handling, logging setup
- Use comments to indicate omitted parts: `# Note: Full lifecycle omitted for clarity`

**Line Count Includes**:
- Class/function definitions
- Docstrings (keep brief)
- Essential logic
- Inline comments (use sparingly)

**Line Count Excludes**:
- Module-level docstrings
- Blank lines for readability
- Explanatory comments below the code block

---

### Python 3.11+ with Type Hints

**Required for All Code**:
- Function signatures: `def callback(self) -> None:`
- Class attributes: `self.count: int = 0`
- Parameter types: `def process(self, data: String) -> None:`
- Return types: always specify, use `None` for void functions

**Rationale**:
- Modern Python best practice
- Helps students understand data flow
- Prepares for professional development
- Type hints are self-documenting

**Example**:
```python
from typing import Optional

class DataProcessor(Node):
    """Processes incoming sensor data."""

    def __init__(self) -> None:
        super().__init__('data_processor')
        self.latest_data: Optional[String] = None
        self.subscriber = self.create_subscription(
            String,
            'sensor_data',
            self.data_callback,
            10
        )

    def data_callback(self, msg: String) -> None:
        """Handle incoming sensor data."""
        self.latest_data = msg
        self.get_logger().info(f'Received: {msg.data}')
```

---

### Clear, Minimal Comments

**Purpose**: Explain WHAT and WHY, not HOW (code shows HOW)

**Comment Guidelines**:
- Class docstrings: 1 sentence explaining purpose
- Method docstrings: Optional, use when behavior isn't obvious
- Inline comments: Only for non-obvious logic
- Avoid: `i = i + 1  # increment i` (obvious)
- Include: `qos_profile = 10  # Queue size for topic buffer` (context)

**Docstring Format**:
```python
class ExampleNode(Node):
    """One-line description of what this node does."""

    def important_method(self, param: int) -> bool:
        """Brief description if method name isn't self-explanatory."""
        # implementation
```

**Don't Over-Comment**:
```python
# BAD - too many comments
class Publisher(Node):
    """Publisher node."""  # Node for publishing

    def __init__(self) -> None:  # Constructor
        super().__init__('pub')  # Initialize parent
        self.pub = self.create_publisher(...)  # Create publisher
```

**Good Balance**:
```python
class Publisher(Node):
    """Publishes sensor readings to the robot_data topic."""

    def __init__(self) -> None:
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(
            SensorData,
            'robot_data',
            10  # Buffer last 10 messages
        )
```

---

### Imports: Only Essentials

**Include**:
- ROS2 imports needed for the example: `from rclpy.node import Node`
- Message types used: `from std_msgs.msg import String`
- Type hints if used: `from typing import Optional`

**Omit**:
- `import rclpy` if not calling `rclpy` functions in snippet
- System libraries not relevant to the concept
- Full import blocks from executable code

**Example for Lesson 3 (rclpy basics)**:
```python
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    # ... implementation
```

---

### No Error Handling or Edge Cases

**Rationale**: Keeps focus on the core pattern

**Omit**:
- Try/except blocks
- Input validation
- Null checks (unless central to the concept)
- Graceful shutdown logic
- Error logging

**Note Below Code**:
Add a comment indicating what's omitted:
```python
# Note: Error handling and full node lifecycle omitted for clarity.
# Production code should include try/except and proper shutdown.
```

---

## Code Structure Patterns

### Pattern 1: ROS2 Node Class (Lessons 3-4)

```python
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """Brief description of node's purpose."""

    def __init__(self) -> None:
        super().__init__('node_name')
        # Setup: publishers, subscribers, timers
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self) -> None:
        """Main logic - publish, process, etc."""
        msg = String()
        msg.data = 'Example data'
        self.publisher.publish(msg)

# Note: Full lifecycle (rclpy.init, spin, shutdown) omitted
```

**Use for**: Publisher examples, subscriber examples, timer examples

---

### Pattern 2: URDF XML Snippet (Lesson 4)

```xml
<!-- Simple humanoid arm with 2 joints -->
<robot name="humanoid_arm">
  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Shoulder joint (revolute) -->
  <joint name="shoulder" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </visual>
  </link>

  <!-- Elbow joint (revolute) -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35"/>
  </joint>
</robot>

<!-- Note: Inertial and collision elements omitted for clarity -->
```

**Use for**: URDF structure examples, joint/link relationships

---

## Explanation Format

**Below Each Code Snippet, Provide**:

1. **Scenario Context** (1 sentence):
   - "This node publishes robot status every second."

2. **Key Components Explanation** (2-3 sentences):
   - "The `__init__` method sets up a publisher and timer. The publisher sends messages to the 'robot_status' topic with a queue size of 10. The timer calls `publish_status()` every second."

3. **Connection to Concepts** (1-2 sentences):
   - "This demonstrates the basic pub/sub pattern: the node publishes messages without knowing who subscribes. Multiple robot components could listen to this status topic."

**Don't Explain**:
- Line-by-line walkthrough (code should be self-explanatory)
- Basic Python syntax (students already know Python)
- How to run the code (not the goal)

---

## Examples by Lesson

### Lesson 3: Python rclpy Bridge

**Example 1: Simple Publisher** (20 lines)
- Demonstrates: Node creation, publisher setup, timer callback, message publishing
- Omits: rclpy.init/shutdown, error handling, command-line args

**Example 2: Simple Subscriber** (18 lines)
- Demonstrates: Subscriber creation, callback function, message handling
- Omits: Full lifecycle, logging setup

**Example 3: Combined Publisher-Subscriber** (28 lines)
- Demonstrates: Both patterns in one node, message flow
- Omits: Complex logic, error handling

### Lesson 4: URDF for Humanoid Robots

**Example 1: 2-Link Robot Arm** (20-25 lines XML)
- Demonstrates: Link and joint definitions, parent-child relationships
- Omits: Inertial properties, collision meshes

**Example 2: Humanoid Torso with Shoulder** (25-30 lines XML)
- Demonstrates: Multiple joints, 3D positioning
- Omits: Full humanoid body, visual meshes

---

## Validation Checklist

Before including code in a lesson:

- [ ] **Length**: 10-30 lines (excluding module docstring, blank lines)
- [ ] **Type hints**: All function signatures and parameters have types
- [ ] **Python version**: Uses Python 3.11+ features (if applicable)
- [ ] **Imports**: Only essential imports included
- [ ] **Docstrings**: Class has one-line docstring
- [ ] **Comments**: Minimal, explain WHY not WHAT
- [ ] **Focus**: Demonstrates ONE concept clearly
- [ ] **Omissions noted**: Comment indicates what's omitted (lifecycle, error handling)
- [ ] **Syntax**: Code is syntactically correct (no errors)
- [ ] **ROS2 version**: Uses ROS2 Humble API patterns
- [ ] **Context**: Explanation below code connects to lesson concepts

---

## Anti-Patterns (What NOT to Do)

❌ **Don't**: Include full executable code with main() and shutdown
✅ **Do**: Show the core pattern only

❌ **Don't**: Add complex error handling and edge cases
✅ **Do**: Focus on the happy path, note omissions

❌ **Don't**: Mix multiple concepts in one snippet
✅ **Do**: One snippet = one concept (publisher OR subscriber)

❌ **Don't**: Explain every line of Python syntax
✅ **Do**: Explain the ROS2-specific parts and overall flow

❌ **Don't**: Use variable names like `x`, `data`, `temp`
✅ **Do**: Use descriptive names like `status_msg`, `sensor_publisher`, `joint_angle`

❌ **Don't**: Show deprecated or ROS1 patterns
✅ **Do**: Use modern ROS2 Humble patterns (rclpy, create_publisher, etc.)

---

## Summary

**The Perfect Code Snippet**:
- **10-30 lines** of clear, focused code
- **Type hints** on all signatures
- **Minimal comments** (docstring + omission note)
- **One concept** demonstrated well
- **Accompanied by** context + explanation connecting to lesson goals
- **Syntactically correct** but not necessarily executable
- **Modern Python** (3.11+) and ROS2 Humble patterns

**Remember**: The goal is conceptual understanding, not hands-on execution. Code should clarify ideas, not burden students with setup complexity.
