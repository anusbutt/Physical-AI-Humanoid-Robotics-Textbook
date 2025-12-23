# Callout Patterns - Module 1: ROS2 Nervous System

**Purpose**: Guidelines for using emoji callouts in Module 1 lessons
**Based on**: research.md decision "Emoji Callout System (💬 🎓 🤝)"
**Target**: CS students with Python knowledge
**Usage**: Include 2-3 callouts per lesson, distributed throughout content

---

## Overview

Callouts are **visual engagement elements** that break up text-heavy content and signal different types of student interaction. Each lesson should include **2-3 callouts** using the three patterns below.

**Why Callouts Work**:
- Visual markers improve retention and scanning
- Break monotony of continuous prose
- Encourage active learning and exploration
- Leverage AI tools for deeper understanding

---

## 💬 AI Colearning Prompt

**Purpose**: Encourage students to explore concepts with AI assistants (Claude, ChatGPT)

**When to Use**:
- After introducing a new concept (deepen understanding)
- When comparing/contrasting ideas (explore nuances)
- For open-ended design challenges (apply knowledge)
- To bridge current lesson to next topic (preview)

**Format**:
```markdown
### 💬 AI Colearning Prompt

> **[Question or Task Title]**: [Open-ended question or design challenge]

[Optional 1-2 sentence explanation of what students should explore]
```

**Content Guidelines**:
- Frame as an open-ended question or design task
- Require applying concepts from THIS lesson
- Should NOT have simple yes/no answers
- Can combine with prior knowledge or real-world scenarios
- Encourage critical thinking or synthesis

**Length**: 1-3 sentences for the prompt itself

---

### Example 1: Lesson 1 (ROS2 Fundamentals)

```markdown
### 💬 AI Colearning Prompt

> **Explore Real-World Analogies**: Ask Claude or ChatGPT to explain the ROS2 pub/sub model using a real-world analogy (like a radio station, newspaper, or social media feed). Which analogy helps you understand it best, and why?
```

**Why This Works**:
- Open-ended (multiple valid analogies)
- Requires understanding pub/sub to evaluate analogies
- Encourages metacognition (which explanation works for ME?)

---

### Example 2: Lesson 2 (Nodes, Topics, Services)

```markdown
### 💬 AI Colearning Prompt

> **Design a Communication Pattern**: Ask Claude to help you design the communication architecture for a simple delivery robot. Should the robot's obstacle sensor use a topic or a service? What about the command to "go to location X"? Explain your reasoning.
```

**Why This Works**:
- Applies topic vs service distinction to new scenario
- Requires justification (not just answers)
- Contextualizes learning in practical robotics

---

### Example 3: Lesson 3 (Python rclpy Bridge)

```markdown
### 💬 AI Colearning Prompt

> **Explain the Callback Pattern**: Ask Claude to explain why ROS2 subscribers use callbacks instead of polling. What are the advantages for robot control systems?
```

**Why This Works**:
- Deepens understanding of design rationale
- Connects to broader CS concepts (event-driven programming)
- Appropriate for students with Python background

---

### Example 4: Lesson 4 (URDF for Humanoid Robots)

```markdown
### 💬 AI Colearning Prompt

> **Trace a Kinematic Tree**: Ask Claude to help you understand how a humanoid robot's URDF kinematic tree works. If the robot moves its shoulder joint, which links move? If it moves the elbow, what stays fixed?
```

**Why This Works**:
- Clarifies parent-child relationships through examples
- Visual/spatial reasoning with AI guidance
- Builds on URDF concepts just learned

---

## 🎓 Expert Insight

**Purpose**: Provide professional context, advanced perspectives, or warn about common mistakes

**When to Use**:
- After explaining a concept (add expert perspective)
- When introducing industry best practices
- To warn about common beginner mistakes
- To provide historical context or design rationale
- To preview advanced topics (without overwhelming)

**Format**:
```markdown
### 🎓 Expert Insight: [Insight Title]

[2-4 paragraphs providing professional context, best practices, or pitfall warnings]
```

**Content Guidelines**:
- Keep accessible (assume beginner knowledge, but add depth)
- Focus on ONE insight (not multiple topics)
- Can reference industry practice, research findings, or common errors
- Explain WHY things work this way (design rationale)
- Avoid overwhelming with jargon (or define terms used)

**Length**: 2-4 paragraphs (150-250 words)

---

### Example 1: Lesson 1 (ROS2 Fundamentals)

```markdown
### 🎓 Expert Insight: ROS2 vs ROS1 - Why the Rewrite?

ROS2 was built from the ground up to address critical limitations in ROS1. The most significant change: **eliminating the master node**. In ROS1, a single "roscore" master coordinated all communication, creating a single point of failure. If the master crashed, the entire robot system failed.

ROS2 uses DDS (Data Distribution Service), a peer-to-peer discovery protocol where nodes find each other automatically without a central coordinator. This makes robot systems more resilient—if one node fails, the rest continue operating.

Another major improvement: **real-time support**. ROS1 struggled with deterministic timing, problematic for safety-critical robots like autonomous vehicles. ROS2 supports real-time operating systems and provides Quality of Service (QoS) policies to guarantee message delivery timing.

**Why this matters for you**: Understanding these design decisions helps you appreciate when to use topics (asynchronous, like ROS1 did) versus services (synchronous requests) and how QoS policies prevent message loss in critical systems.
```

**Why This Works**:
- Contextualizes ROS2 design decisions
- Explains WHY (not just WHAT)
- Accessible to beginners but adds depth
- Previews concepts (QoS) without requiring full understanding yet

---

### Example 2: Lesson 2 (Nodes, Topics, Services)

```markdown
### 🎓 Expert Insight: QoS Policies - When Message Delivery Matters

When you create a publisher or subscriber, you specify a QoS (Quality of Service) profile. The number `10` you've seen in code examples (e.g., `create_publisher(String, 'topic', 10)`) is actually a queue size setting—how many messages to buffer if the subscriber can't keep up.

But QoS goes deeper. For a camera streaming 30 fps video, you want "best effort" reliability—it's okay to drop old frames. But for a motor controller receiving position commands, you need "reliable" delivery—every command must arrive in order.

ROS2 provides pre-configured QoS profiles:
- **Sensor Data**: Best effort, volatile (okay to drop messages)
- **System Default**: Reliable, volatile (try to deliver all)
- **Services**: Reliable, volatile (guaranteed request/response)

**Common pitfall**: Mismatched QoS between publisher and subscriber can cause silent failures. If a publisher uses "best effort" but subscriber expects "reliable," they won't communicate. Always verify QoS compatibility when debugging connection issues.
```

**Why This Works**:
- Explains a confusing detail (that `10` number)
- Provides practical guidance (when to use which QoS)
- Warns about common error (mismatched QoS)
- Appropriate depth for students learning topics/services

---

### Example 3: Lesson 3 (Python rclpy Bridge)

```markdown
### 🎓 Expert Insight: Type Hints in ROS2 Code

You'll notice modern ROS2 tutorials use Python type hints (`def callback(self, msg: String) -> None:`). This isn't just good style—it's increasingly important for robot development.

**Why type hints matter in robotics**:
1. **Catch errors early**: Type checkers (like mypy) can detect message type mismatches before runtime
2. **Self-documenting**: `create_subscription(String, ...)` clearly shows what message type this subscriber expects
3. **IDE support**: Modern editors auto-complete message fields when types are specified

**Best practice**: Always specify message types in your node signatures. It takes 5 extra seconds but prevents hours of debugging cryptic runtime errors like "AttributeError: 'Int32' object has no attribute 'data'".

**Looking ahead**: When you work with custom message types (not just `String`), type hints become essential for understanding what data flows through your robot system.
```

**Why This Works**:
- Explains WHY a coding convention matters (not just "do it")
- Practical benefits students care about (fewer bugs, better IDE support)
- Appropriate for students with Python background
- Previews custom messages without full explanation

---

### Example 4: Lesson 4 (URDF for Humanoid Robots)

```markdown
### 🎓 Expert Insight: URDF Units and Coordinate Frames

A common mistake when starting with URDF: **forgetting that all dimensions use meters and radians**, not centimeters or degrees. If your robot looks tiny in simulation, you probably specified link lengths in centimeters.

**Coordinate frame conventions**:
- **X-axis**: Forward (robot's direction of motion)
- **Y-axis**: Left
- **Z-axis**: Up
- **Rotations**: Right-hand rule (counterclockwise is positive)

These conventions matter when defining joint axes. A shoulder joint rotating around the Y-axis (`<axis xyz="0 1 0"/>`) raises the arm up/down. The same joint around the Z-axis would rotate the arm forward/back.

**Pro tip**: When debugging weird simulation behavior, verify your units first (meters not cm, radians not degrees), then check coordinate frames (is Z pointing up?). These two issues cause 90% of beginner URDF problems.

**Why URDF uses radians**: Math libraries (sin, cos for kinematics) expect radians. Using degrees throughout would require constant conversion, slowing real-time control loops.
```

**Why This Works**:
- Addresses common beginner mistakes
- Explains conventions clearly
- Provides debugging guidance
- Includes practical rationale (why radians?)

---

## 🤝 Practice Exercise

**Purpose**: Give students a hands-on task to apply what they've learned

**When to Use**:
- After explaining key concepts (apply immediately)
- Near end of lesson (consolidate understanding)
- To prepare for next lesson (bridge concepts)
- As formative assessment (check understanding)

**Format**:
```markdown
### 🤝 Practice Exercise: [Exercise Title]

**Task**: [Clear description of what to do]

[2-4 sentences providing guidance, hints, or structure]

**Consider**:
- [Guiding question 1]
- [Guiding question 2]
- [Guiding question 3]
```

**Content Guidelines**:
- Should be completable in **10-15 minutes**
- Can be conceptual (diagram, design) OR experimental (modify code)
- Provide enough structure that students know where to start
- Does NOT require local ROS2 installation (use design/diagrams)
- Can suggest using AI tools for validation/feedback

**Length**: 3-5 sentences for task + 2-4 guiding questions

---

### Example 1: Lesson 1 (ROS2 Fundamentals)

```markdown
### 🤝 Practice Exercise: Design a Robot Communication Graph

**Task**: Sketch a simple ROS2 graph for a humanoid robot doing a household task (like fetching a cup).

Identify at least **3 nodes** (e.g., vision system, arm controller, navigation) and **2 topics** for communication between them. Label your diagram with node names and topic names.

**Consider**:
- What sensor data needs to flow through the system?
- Which nodes publish data, and which subscribe?
- How does information get from perception (seeing the cup) to action (moving the arm)?

**Optional**: Share your diagram with Claude and ask: "Does this communication pattern make sense for a fetch task?"
```

**Why This Works**:
- Applies pub/sub concepts immediately
- Concrete, relatable scenario
- Open-ended (multiple valid solutions)
- Doesn't require installation
- Optional AI validation step

---

### Example 2: Lesson 2 (Nodes, Topics, Services)

```markdown
### 🤝 Practice Exercise: Topics vs Services Decision

**Task**: For each robot scenario below, decide whether to use a **topic** (pub/sub) or **service** (request/response). Explain your reasoning.

1. **Camera streaming** 30 frames per second to object detection
2. **Emergency stop button** that must immediately halt all motors
3. **Arm controller** requesting the next target position from a planner
4. **Battery level** monitor broadcasting charge percentage every second

**Consider**:
- Is data flowing continuously or on-demand?
- Is the requester waiting for a response, or fire-and-forget?
- How many publishers and subscribers might be involved?

**Validate**: Ask Claude if your decisions match ROS2 best practices.
```

**Why This Works**:
- Directly applies topic vs service distinction
- Multiple scenarios (not just one)
- Requires justification (deeper understanding)
- AI validation available

---

### Example 3: Lesson 3 (Python rclpy Bridge)

```markdown
### 🤝 Practice Exercise: Trace the Callback Flow

**Task**: Look at the simple subscriber code example from this lesson. Trace what happens when a message arrives on the topic.

Write out the **sequence of events** in plain English:
1. A message is published to 'sensor_data' topic
2. ROS2 middleware detects a new message
3. [What happens next?]
4. [And then?]
5. [Final step?]

**Consider**:
- When does the callback function get called?
- What happens to `self.latest_data`?
- Is the callback synchronous or asynchronous?

**Challenge**: Modify the example to publish a response message whenever data is received. What changes?
```

**Why This Works**:
- Encourages tracing code execution (important skill)
- Checks understanding of asynchronous callbacks
- Optional extension for advanced students
- No installation required (conceptual exercise)

---

### Example 4: Lesson 4 (URDF for Humanoid Robots)

```markdown
### 🤝 Practice Exercise: Sketch a Humanoid Robot Leg

**Task**: Design the URDF structure for a simplified humanoid robot leg with **3 joints**: hip, knee, and ankle.

For each component, identify:
- **Links**: What are the rigid body parts? (thigh, shin, foot)
- **Joints**: What type? (revolute, prismatic, fixed)
- **Parent-child relationships**: Which link connects to which?

**Consider**:
- What's the root link (where does the leg attach to the body)?
- Which joints allow rotation (revolute) vs extension (prismatic)?
- What axis does each joint rotate around (X, Y, or Z)?

**Optional**: Sketch your kinematic tree on paper, then ask Claude if the parent-child relationships are correct for a walking motion.
```

**Why This Works**:
- Applies URDF concepts to new body part (transfer learning)
- Requires understanding link/joint relationships
- Concrete output (sketch/diagram)
- AI validation step
- Prepares for capstone project

---

## Distribution Guidelines

### Per Lesson (2-3 Callouts Total)

**Recommended Pattern**:
- **Early in lesson**: 💬 AI Colearning (deepen new concept)
- **Middle of lesson**: 🎓 Expert Insight (add context/warn about pitfalls)
- **Near end of lesson**: 🤝 Practice Exercise (apply knowledge)

**Alternative Pattern**:
- 💬 AI Colearning (early)
- 🎓 Expert Insight (middle)
- 💬 AI Colearning (late - bridge to next lesson)

**Don't Overuse**: More than 3 callouts per lesson dilutes impact

---

### Placement in Content Flow

**💬 AI Colearning**:
- After "What Is [Concept]?" section (explore deeper)
- After "Key Principles" (apply to scenarios)
- Before "Next Steps" (preview next topic)

**🎓 Expert Insight**:
- After "Why [Concept] Matters" (add industry context)
- Within "Key Principles" (explain design rationale)
- After "Practical Example" (discuss best practices)

**🤝 Practice Exercise**:
- After "Practical Example" (apply immediately)
- After "Key Principles" (consolidate understanding)
- Before "Summary" (check learning)

---

## Validation Checklist

Before finalizing callouts in a lesson:

- [ ] **Count**: 2-3 callouts per lesson (not more, not fewer)
- [ ] **Distribution**: Spread throughout lesson (not all bunched)
- [ ] **At least one 💬**: Every lesson has AI Colearning prompt
- [ ] **Open-ended questions**: No yes/no questions in 💬 callouts
- [ ] **Appropriate depth**: 🎓 insights accessible to beginners
- [ ] **Actionable exercises**: 🤝 tasks completable in 10-15 min
- [ ] **No installation required**: Exercises don't need local ROS2 setup
- [ ] **Emoji correct**: Using 💬 🎓 🤝 (not other emoji)
- [ ] **Technical Reviewer**: Content validated for accuracy

---

## Anti-Patterns (What NOT to Do)

❌ **Don't**: Use 5+ callouts per lesson (overwhelming)
✅ **Do**: Limit to 2-3 high-impact callouts

❌ **Don't**: Ask "What is ROS2?" in AI prompt (simple lookup question)
✅ **Do**: Ask open-ended design or comparison questions

❌ **Don't**: Write 6-paragraph Expert Insights (too dense)
✅ **Do**: Keep insights focused (2-4 paragraphs, one topic)

❌ **Don't**: Give exercises requiring ROS2 installation
✅ **Do**: Focus on conceptual/design exercises

❌ **Don't**: Use random emoji (🔥🚀⚡)
✅ **Do**: Stick to the 3 defined patterns (💬🎓🤝)

---

## Summary

**The Perfect Callout Set** (per lesson):
- **2-3 callouts** distributed throughout
- **💬 AI Colearning**: Open-ended, requires applying concepts
- **🎓 Expert Insight**: Adds depth without overwhelming, 2-4 paragraphs
- **🤝 Practice Exercise**: 10-15 min, concrete task, no installation needed
- **Strategically placed**: Enhance flow, don't interrupt it
- **Validated**: Technical accuracy confirmed by Technical Reviewer Agent

**Remember**: Callouts are engagement tools, not content dumps. Each should have a clear purpose and add value beyond the main prose.
