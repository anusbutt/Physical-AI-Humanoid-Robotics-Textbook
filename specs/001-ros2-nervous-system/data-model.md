# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-05
**Phase**: 1 (Design)
**Purpose**: Define the structure of lesson content entities, their relationships, and validation rules

## Overview

This data model defines the structure of educational content for Module 1. Unlike traditional software data models (database schemas, API entities), this model describes **content artifacts** (markdown files, YAML metadata, code snippets) and their relationships.

## Core Entities

### 1. Lesson

A complete educational unit covering one ROS2 concept.

**Attributes**:
- `lesson_number`: Integer (1-4)
- `title`: String (e.g., "ROS2 Fundamentals")
- `filename_base`: String (e.g., "01-ros2-fundamentals")
- `main_content`: MarkdownDocument (`.md` file)
- `summary_content`: MarkdownDocument (`.summary.md` file)
- `frontmatter`: FrontmatterMetadata (YAML block)
- `code_examples`: Array<CodeSnippet> (0-5 snippets per lesson)
- `callouts`: Array<Callout> (2-3 per lesson)
- `word_count`: Integer (1500-2500 words)
- `estimated_time`: Integer (minutes, 30-45)

**Relationships**:
- Lesson belongs_to Module
- Lesson has_many CodeSnippets
- Lesson has_many Callouts
- Lesson has_one FrontmatterMetadata

**Validation Rules**:
- `title` must be unique within module
- `filename_base` must match pattern `##-topic-name` (two-digit prefix)
- `main_content` must follow 7-section template (research.md)
- `summary_content` must exist and contain 3-5 bullet points
- `word_count` must be 1500-2500 (FR-007: conceptual snippets, not full tutorials)
- `estimated_time` must be ≤45 minutes (SC-001)

**File Mapping**:
```
Lesson 1 → {
  main_content: "01-ros2-fundamentals.md"
  summary_content: "01-ros2-fundamentals.summary.md"
}
```

---

### 2. Module

A collection of 4 related lessons.

**Attributes**:
- `module_number`: Integer (1)
- `title`: String ("The Robotic Nervous System (ROS 2)")
- `directory_name`: String ("01-ros2-nervous-system")
- `lessons`: Array<Lesson> (exactly 4)
- `readme_content`: MarkdownDocument (`README.md`)
- `capstone_project`: CapstoneProject
- `quiz`: Quiz

**Relationships**:
- Module has_many Lessons (exactly 4)
- Module has_one CapstoneProject
- Module has_one Quiz
- Module belongs_to Book

**Validation Rules**:
- Must contain exactly 4 lessons
- `directory_name` must match pattern `##-module-name-kebab-case`
- `readme_content` must include overview and navigation links to all lessons
- `capstone_project` and `quiz` must exist

**File Structure**:
```
01-ros2-nervous-system/
├── README.md                          (module.readme_content)
├── 01-ros2-fundamentals.md            (lessons[0].main_content)
├── 01-ros2-fundamentals.summary.md    (lessons[0].summary_content)
├── 02-nodes-topics-services.md        (lessons[1].main_content)
├── 02-nodes-topics-services.summary.md
├── 03-python-rclpy-bridge.md          (lessons[2].main_content)
├── 03-python-rclpy-bridge.summary.md
├── 04-urdf-humanoid-basics.md         (lessons[3].main_content)
├── 04-urdf-humanoid-basics.summary.md
├── 05-capstone-project.md             (module.capstone_project.content)
├── 05-capstone-project.summary.md
└── 06-quiz.md                         (module.quiz.content)
```

---

### 3. FrontmatterMetadata

YAML metadata block at the top of each lesson file.

**Attributes**:
- `title`: String
- `sidebar_position`: Integer (1-4 for lessons, 5 for capstone, 6 for quiz)
- `skills`: Array<Skill>
- `learning_objectives`: Array<LearningObjective>
- `cognitive_load`: CognitiveLoadAssessment
- `differentiation`: DifferentiationGuidance
- `tags`: Array<String>
- `generated_by`: Enum("manual", "agent")
- `created`: ISO Date String (YYYY-MM-DD)
- `last_modified`: ISO Date String
- `ros2_version`: String ("humble")

**Nested Entities**:

**Skill**:
- `name`: String (e.g., "ROS2 Fundamentals")
- `proficiency_level`: Enum("beginner", "intermediate", "advanced")
- `category`: String (e.g., "robotics-middleware")
- `bloom_level`: Enum("remember", "understand", "apply", "analyze", "evaluate", "create")
- `digcomp_area`: String (e.g., "technical-concepts")
- `measurable_at_this_level`: String (e.g., "explain pub/sub model")

**LearningObjective**:
- `objective`: String (e.g., "Understand ROS2 nodes and topics")
- `proficiency_level`: Enum("beginner", "intermediate", "advanced")
- `bloom_level`: Enum("remember", "understand", "apply", "analyze", "evaluate", "create")
- `assessment_method`: String (e.g., "quiz + capstone project")

**CognitiveLoadAssessment**:
- `new_concepts`: Integer (3-7)
- `assessment`: String (e.g., "moderate - builds on Python knowledge")

**DifferentiationGuidance**:
- `extension_for_advanced`: String (e.g., "Explore ROS2 services and actions")
- `remedial_for_struggling`: String (e.g., "Review Python basics first")

**Validation Rules**:
- All fields required (no null/empty values)
- `sidebar_position` must be unique within module
- `skills` must contain 1-3 skill objects
- `learning_objectives` must contain 1-5 objectives
- `cognitive_load.new_concepts` must be 3-7 (per lesson)
- `bloom_level` must align with Bloom's taxonomy
- Dates must be ISO format (YYYY-MM-DD)

**YAML Example**:
```yaml
---
title: "ROS2 Fundamentals"
sidebar_position: 1
skills:
  - name: "ROS2 Architecture Understanding"
    proficiency_level: "beginner"
    category: "robotics-middleware"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "explain ROS2's role as middleware"
learning_objectives:
  - objective: "Understand what ROS2 is and why it's used in robotics"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz + discussion"
cognitive_load:
  new_concepts: 5
  assessment: "moderate - builds on networking and Python knowledge"
differentiation:
  extension_for_advanced: "Research ROS2 DDS implementations and QoS profiles"
  remedial_for_struggling: "Review distributed systems basics first"
tags: ["ros2", "middleware", "fundamentals", "architecture"]
generated_by: "manual"
created: "2025-12-05"
last_modified: "2025-12-05"
ros2_version: "humble"
---
```

---

### 4. CodeSnippet

Python code example demonstrating ROS2/rclpy/URDF concept.

**Attributes**:
- `language`: String ("python" or "xml")
- `code`: String (10-30 lines)
- `explanation`: String (what the code does conceptually)
- `context`: String (where this appears in lesson)
- `has_type_hints`: Boolean (true for Python)
- `is_executable`: Boolean (false - conceptual only)

**Validation Rules**:
- `code` must be 10-30 lines (FR-007)
- Python code must include type hints (FR-007)
- Must be syntactically valid (automated check)
- Must not include complex error handling or full boilerplate (conceptual focus)

**Example**:
```python
# Language: python
# Context: Lesson 3 - Creating a simple publisher

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """Publishes status messages to a ROS2 topic."""

    def __init__(self) -> None:
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self) -> None:
        msg = String()
        msg.data = 'Robot operational'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

**Explanation**: "This snippet demonstrates the core pattern for creating a ROS2 publisher using rclpy. The node publishes a status message every second to the 'robot_status' topic."

---

### 5. Callout

Interactive learning element (AI prompt, expert insight, practice exercise).

**Attributes**:
- `type`: Enum("ai_colearning", "expert_insight", "practice_exercise")
- `emoji`: String ("💬", "🎓", "🤝")
- `title`: String (optional)
- `content`: String (1-3 paragraphs)
- `context`: String (which section of lesson)

**Validation Rules**:
- Each lesson must have 2-3 callouts (distributed throughout)
- At least 1 `ai_colearning` callout per lesson
- `content` must be appropriate for student level (Technical Reviewer validation)

**Example**:
```markdown
### 💬 AI Colearning Prompt

> Ask Claude or ChatGPT: "Explain the difference between a ROS2 topic and a ROS2 service using a real-world analogy. Which would you use for streaming sensor data, and which for requesting a robot to move to a specific position?"
```

---

### 6. CapstoneProject

Integrative hands-on exercise combining all module concepts.

**Attributes**:
- `title`: String ("Design a Simple Delivery Robot System")
- `description`: String (project overview)
- `requirements`: Array<String> (what students must deliver)
- `guidance`: String (hints, example structure)
- `evaluation_rubric`: Array<RubricCriteria>
- `estimated_time`: Integer (60-120 minutes)

**RubricCriteria**:
- `criterion`: String (e.g., "Node architecture design")
- `points`: Integer (e.g., 25)
- `description`: String (what earns full points)

**Validation Rules**:
- Must integrate all 4 lesson concepts (ROS2, nodes, rclpy, URDF)
- Must be conceptual design (not full implementation)
- `estimated_time` must be ≤120 minutes (SC-004: minimal help requirement)

**File**: `05-capstone-project.md`

---

### 7. Quiz

Assessment with multiple question types.

**Attributes**:
- `total_questions`: Integer (15)
- `passing_score`: Integer (12/15 = 80%, aligns with SC-001)
- `questions`: Array<QuizQuestion>
- `answer_key`: Array<Answer>
- `grading_rubric`: String

**QuizQuestion**:
- `question_number`: Integer (1-15)
- `type`: Enum("multiple_choice", "short_answer", "code_reading")
- `text`: String (question prompt)
- `options`: Array<String> (for multiple_choice only)
- `correct_answer`: String or Integer (index for MC)
- `explanation`: String (why this answer is correct)

**Validation Rules**:
- Must have exactly 15 questions
- Must include mix of types (8 MC, 5 short answer, 2 code reading)
- All questions must map to specific learning objectives from lessons

**File**: `06-quiz.md`

---

## Entity Relationships Diagram

```
Book
  └── Module (Module 1)
      ├── Lesson 1
      │   ├── FrontmatterMetadata
      │   ├── CodeSnippet (0-5)
      │   └── Callout (2-3)
      ├── Lesson 2
      │   ├── FrontmatterMetadata
      │   ├── CodeSnippet (0-5)
      │   └── Callout (2-3)
      ├── Lesson 3
      │   ├── FrontmatterMetadata
      │   ├── CodeSnippet (0-5)
      │   └── Callout (2-3)
      ├── Lesson 4
      │   ├── FrontmatterMetadata
      │   ├── CodeSnippet (0-5)
      │   └── Callout (2-3)
      ├── CapstoneProject
      └── Quiz
```

---

## Validation Summary

**Automated Validations** (run before commit):
1. YAML schema validation (frontmatter matches structure)
2. Python syntax check (code snippets parse correctly)
3. Markdown linting (headings, links, formatting)
4. File naming convention check (`##-topic.md` + `.summary.md` pairs exist)
5. Word count check (1500-2500 per lesson)

**Agent Validations** (Technical Reviewer):
1. ROS2 concept accuracy
2. rclpy API correctness
3. URDF syntax validation
4. Pedagogical appropriateness (explanations match student level)
5. Callout quality (prompts are clear and appropriate)

**Manual Validations** (user review):
1. Content matches learning objectives
2. Examples clarify concepts effectively
3. Flow between lessons is logical
4. Capstone integrates all concepts

---

## Implementation Notes

- All entities are represented as markdown files (not database records)
- Frontmatter metadata stored as YAML blocks (parsed by Docusaurus)
- Relationships enforced by file structure and naming conventions
- Validation performed via scripts + Technical Reviewer Agent
- Data model guides content creation but is not rigidly enforced (educational content requires flexibility)
