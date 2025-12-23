# Agent Contract: Outline Agent

**Agent Type**: Authoring Agent
**Purpose**: Generate structured subsection outlines for lessons based on topic and learning goals
**Status**: To be implemented in Iteration 3 (Lesson 4) if pattern stabilizes

## Input Schema

```yaml
topic:
  type: string
  required: true
  description: "Lesson topic (e.g., 'ROS2 Fundamentals', 'Python rclpy Bridge')"
  example: "ROS2 Fundamentals"

learning_goals:
  type: array<string>
  required: true
  description: "What students should learn from this lesson"
  min_items: 2
  max_items: 5
  example:
    - "Understand what ROS2 is and its role as middleware"
    - "Explain the ROS2 pub/sub model conceptually"
    - "Identify core ROS2 components (nodes, topics, graph)"

audience_context:
  type: object
  required: false
  properties:
    prior_knowledge:
      type: array<string>
      description: "What students already know"
      example: ["Python OOP", "basic networking"]
    difficulty_level:
      type: string
      enum: ["beginner", "intermediate", "advanced"]
      default: "beginner"
```

## Output Schema

```yaml
type: markdown
structure: |
  # [Topic Title]

  ## What Is [Concept]?
  - [Key point 1]
  - [Key point 2]

  ## Why [Concept] Matters for Physical AI
  - [Relevance point 1]
  - [Relevance point 2]

  ### Key Principles
  1. [Principle 1]
  2. [Principle 2]
  3. [Principle 3]

  ### 💬 AI Colearning Prompt
  - [Suggested prompt for students]

  ### 🎓 Expert Insight
  - [Advanced perspective or pitfall]

  ## Practical Example
  - [What example will demonstrate]
  - [Key code concepts to show]

  ### 🤝 Practice Exercise
  - [Hands-on task description]

  ## Summary
  - [Takeaway 1]
  - [Takeaway 2]
  - [Takeaway 3]

  ## Next Steps
  - [Preview of next lesson]

format: "Bullet points under each H2/H3 heading; full prose filled in by Content Agent"
sections: 7
headings_h2: 4
headings_h3: 4
```

## Acceptance Criteria

- [ ] Output follows 7-section template exactly (What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps)
- [ ] All H2 headings present (4 required)
- [ ] All H3 headings present (4 required: Key Principles, AI Prompt, Expert Insight, Practice Exercise)
- [ ] Emoji callouts use correct symbols (💬, 🎓, 🤝)
- [ ] Bullet points are concise placeholders (not full paragraphs)
- [ ] Learning goals from input are distributed across sections appropriately
- [ ] No placeholder text like [TODO], [FILL], [TBD]
- [ ] Output is valid Markdown (parseable by Docusaurus)
- [ ] Section order matches research.md pattern

## Failure Modes

| Failure Mode | Trigger | Recovery Strategy |
|--------------|---------|------------------|
| **Missing H2 heading** | Output lacks "What Is", "Why Matters", "Practical Example", or "Summary" | Retry with explicit section checklist in prompt |
| **Incorrect emoji** | Uses wrong emoji or no emoji for callouts | Auto-fix: Replace with correct emoji (💬/🎓/🤝) |
| **Too detailed** | Bullet points are full paragraphs instead of concise placeholders | Retry with instruction: "Generate only bullet point placeholders, not full content" |
| **Missing learning goal coverage** | Outline doesn't address all input learning goals | Validate each goal appears in at least one section; retry if missing |
| **Invalid Markdown** | Syntax errors (broken headings, unescaped characters) | Run markdown linter, auto-fix common issues, retry if unfixable |

## Usage Example

**Input**:
```yaml
topic: "ROS2 Fundamentals"
learning_goals:
  - "Understand what ROS2 is and its role as middleware"
  - "Explain the ROS2 pub/sub model conceptually"
  - "Identify core ROS2 components (nodes, topics, graph)"
audience_context:
  prior_knowledge: ["Python OOP", "basic networking"]
  difficulty_level: "beginner"
```

**Expected Output** (excerpt):
```markdown
# ROS2 Fundamentals

## What Is ROS2?
- Definition: Robot Operating System 2 as middleware
- Distributed systems architecture
- Comparison to traditional monolithic robot software

## Why ROS2 Matters for Physical AI
- Modularity enables complex robot behaviors
- Real-time communication for sensor/actuator coordination
- Hardware abstraction across different robots
...
```

## Implementation Notes

- First iteration (Lesson 1): Manual outline creation following template
- Second iteration (Lessons 2-3): Copy-paste-modify from Lesson 1
- Third iteration (Lesson 4+): Implement agent if pattern stabilizes
- Agent implementation: Claude Code Task tool with specialized prompt
- Validation: Automated Markdown check + manual review
