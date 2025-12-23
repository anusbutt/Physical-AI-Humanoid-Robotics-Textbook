# Lesson Content Template - Module 1: ROS2 Nervous System

**Purpose**: Standard template for creating Module 1 lesson content
**Based on**: 7-section pattern from research.md
**Target**: Educational content for CS students with Python knowledge
**Usage**: Follow this structure for all 4 lessons in Module 1

---

## Template Structure

```markdown
---
title: "[Lesson Title - What Students Will Learn]"
sidebar_position: [1-6]
skills:
  - name: "[Skill Name]"
    proficiency_level: "beginner" | "intermediate" | "advanced"
    category: "[e.g., robotics-middleware, programming, system-design]"
    bloom_level: "remember" | "understand" | "apply" | "analyze" | "evaluate" | "create"
    digcomp_area: "[e.g., technical-concepts, problem-solving]"
    measurable_at_this_level: "[What student can demonstrate, e.g., 'explain pub/sub model']"
learning_objectives:
  - objective: "[What student will be able to do]"
    proficiency_level: "beginner" | "intermediate" | "advanced"
    bloom_level: "remember" | "understand" | "apply" | "analyze" | "evaluate" | "create"
    assessment_method: "[How measured, e.g., 'quiz + capstone project']"
cognitive_load:
  new_concepts: [3-7]
  assessment: "[e.g., 'moderate - builds on Python knowledge']"
differentiation:
  extension_for_advanced: "[Optional deeper exploration for advanced students]"
  remedial_for_struggling: "[Prerequisite review or simpler explanation]"
tags: ["ros2", "middleware", "[topic-specific-tags]"]
generated_by: "manual" | "agent"
created: "YYYY-MM-DD"
last_modified: "YYYY-MM-DD"
ros2_version: "humble"
---

# [Lesson Title]

[Brief 1-2 sentence introduction to the lesson topic]

## What Is [Concept]?

[400-500 words]

**Purpose**: Define the concept clearly and provide context

**Content Guidelines**:
- Start with a simple, jargon-free definition
- Explain key terminology on first use
- Use analogies to connect to students' existing knowledge (Python, networking, etc.)
- Avoid implementation details - focus on conceptual understanding
- Include comparison to familiar systems if applicable (e.g., ROS2 vs traditional software)

**Structure**:
- Opening definition (1-2 sentences)
- Expanded explanation with context (2-3 paragraphs)
- Relationship to physical AI/humanoid robotics (1 paragraph)

---

## Why [Concept] Matters for Physical AI

[400-500 words]

**Purpose**: Motivate learning by showing real-world relevance

**Content Guidelines**:
- Connect to humanoid robotics use cases
- Explain practical benefits (modularity, scalability, etc.)
- Show how this enables complex robot behaviors
- Reference real-world examples (without requiring deep domain knowledge)
- Avoid overly technical justifications - keep it accessible

**Structure**:
- Problem statement: What challenge does this solve? (1 paragraph)
- Benefits and capabilities (2-3 paragraphs)
- Future implications for physical AI (1 paragraph)

---

### Key Principles

[3-5 enumerated core concepts]

**Purpose**: Break down the concept into digestible ideas

**Content Guidelines**:
- Each principle should be 1-3 sentences
- Use clear, declarative statements
- Include brief explanations or examples
- Order from fundamental to advanced
- Relate to students' Python knowledge where possible

**Example Format**:
1. **[Principle Name]**: [Brief explanation with example]
2. **[Principle Name]**: [Brief explanation with example]
3. **[Principle Name]**: [Brief explanation with example]

---

### 💬 AI Colearning Prompt

> **Suggested Exploration**: [Question or task students can explore with Claude/ChatGPT]

**Purpose**: Encourage deeper exploration with AI tools

**Content Guidelines**:
- Frame as an open-ended question or design challenge
- Should require applying concepts from this lesson
- Can combine with prior knowledge
- Avoid questions with simple yes/no answers
- Example: "Ask Claude to explain [concept] using a real-world analogy. Which would you use for [scenario A], and which for [scenario B]?"

---

### 🎓 Expert Insight

**[Insight Title - e.g., "ROS2 vs ROS1 Differences" or "Common Pitfall: QoS Policies"]**

[2-4 paragraphs providing advanced perspective]

**Purpose**: Offer professional context or warn about common mistakes

**Content Guidelines**:
- Share best practices from industry/research
- Highlight common misconceptions or errors
- Provide deeper technical context (without overwhelming beginners)
- Can include performance considerations, design patterns, or historical context
- Keep accessible - assume beginner-level understanding

---

## Practical Example

[300-400 words + optional code snippet]

**Purpose**: Demonstrate the concept with a concrete example

**Content Guidelines**:
- Use a humanoid robot scenario when possible
- If code is included: 10-30 lines, Python 3.11+, type hints, clear comments
- Code should be conceptual, not fully executable
- Explain what the code does, not line-by-line
- Focus on the pattern/structure, not error handling or boilerplate

**Structure**:
- Scenario description (1 paragraph)
- Code snippet (if applicable - for Lessons 3-4)
- Explanation of how it works (1-2 paragraphs)
- Connection back to key principles (1 paragraph)

**Code Example Format** (if applicable):
```python
"""
Brief description of what this code demonstrates
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """Clear docstring explaining the node's purpose."""

    def __init__(self) -> None:
        super().__init__('example_node_name')
        # Explain what this setup does
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self) -> None:
        """Explain what the callback does."""
        msg = String()
        msg.data = 'Example message'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

# Note: This is a conceptual example - omits error handling and full lifecycle
```

---

### 🤝 Practice Exercise

**[Exercise Title - e.g., "Design a Robot Communication Pattern"]**

[Description of the hands-on task]

**Purpose**: Apply concepts through design or experimentation

**Content Guidelines**:
- Should be completable in 10-15 minutes
- Can be conceptual (diagram a system) or experimental (modify code)
- Provide enough guidance that students know where to start
- Avoid requiring local ROS2 installation (conceptual focus)
- Can suggest using AI tools for validation

**Example Format**:
"Diagram a simple [robot system] with [X nodes] and [Y topics/services]. Consider:
- What data needs to flow between components?
- Which communication pattern (topic vs service) is appropriate for each?
- How would you handle [specific scenario]?"

---

## Summary

**Key Takeaways**:
- **[Takeaway 1]**: [1-2 sentence recap of major point]
- **[Takeaway 2]**: [1-2 sentence recap of major point]
- **[Takeaway 3]**: [1-2 sentence recap of major point]
- [Optional: **[Takeaway 4-5]** for complex lessons]

**What You Should Now Understand**:
[1-2 sentences reinforcing the lesson's learning objectives]

---

## Next Steps

[100-150 words]

**Purpose**: Preview the next lesson and show progression

**Content Guidelines**:
- Briefly recap what was covered
- Explain what comes next and why
- Create continuity between lessons
- Generate curiosity about the next topic

**Structure**:
- Current lesson recap (1 sentence)
- Next lesson preview (2-3 sentences)
- How they connect (1-2 sentences)

**Example**:
"You now understand [current concept]. In the next lesson, we'll explore [next concept], which builds on [current concept] to enable [capability]. This progression from [A] to [B] mirrors how real roboticists design [system type]."

---

## Summary File Template (`[lesson-name].summary.md`)

```markdown
# [Lesson Title] - Summary

**Quick Reference**: Key concepts from this lesson

## Core Concept

[1-2 sentence definition of the main concept]

## Key Points

- **[Point 1]**: [One sentence]
- **[Point 2]**: [One sentence]
- **[Point 3]**: [One sentence]
- [Optional: **[Point 4-5]**]

## When to Use

[1-2 sentences explaining practical application]

## Common Patterns

[If applicable - 1-2 sentences about typical usage patterns]

## Related Concepts

- [Concept from previous lesson]
- [Concept from next lesson]

---

**Total word count**: 100-200 words
```

---

## Validation Checklist

Before considering a lesson complete, verify:

**Content Quality**:
- [ ] Word count: 1500-2500 (main lesson), 100-200 (summary)
- [ ] All 7 sections present (What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps)
- [ ] Frontmatter includes all 13 required fields
- [ ] 2-3 callouts distributed throughout (💬 🎓 🤝)
- [ ] Code examples (if present): 10-30 lines, type hints, conceptual focus

**Technical Accuracy** (via Technical Reviewer Agent):
- [ ] ROS2 concepts accurate for Humble distribution
- [ ] rclpy API usage correct (Lessons 3-4)
- [ ] URDF syntax valid (Lesson 4)
- [ ] No misleading analogies or outdated information

**Pedagogical Quality**:
- [ ] Targets CS students with Python knowledge
- [ ] Jargon explained on first use
- [ ] Appropriate difficulty for beginners
- [ ] Clear learning progression (What → Why → How)
- [ ] Actionable practice exercise

**RAG Readiness** (Principle I):
- [ ] Clear H2/H3 heading hierarchy
- [ ] Semantic chunking (each section is modular)
- [ ] Rich frontmatter metadata
- [ ] Summary file exists and maps to main content

**File Naming**:
- [ ] Main file: `##-topic-name.md` (e.g., `01-ros2-fundamentals.md`)
- [ ] Summary file: `##-topic-name.summary.md`
- [ ] Sidebar position matches file number prefix

---

## Notes

- **Estimated time to write**: 4-6 hours (manual), 2-3 hours (template), 1-2 hours (agent)
- **Target completion time** (student): 30-45 minutes
- **Quiz coverage**: Each lesson maps to 3-4 quiz questions
- **Capstone integration**: Each lesson contributes 1 component to capstone project

**References**:
- 7-section pattern: `research.md`
- Frontmatter schema: `data-model.md`
- Agent contracts: `contracts/outline-agent.md`, `contracts/technical-reviewer-agent.md`
- Quality gates: `quickstart.md`
