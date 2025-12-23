# Research: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-05
**Phase**: 0 (Research & Best Practices)
**Purpose**: Resolve unknowns, establish technical foundations, identify best practices for educational content creation

## ROS2 Educational Content Best Practices

### Decision: ROS2 Humble as Reference Distribution

**Rationale**:
- ROS2 Humble is the current LTS (Long Term Support) release (supported until 2027)
- Widely adopted in industry and academia
- Stable API reduces risk of code examples becoming outdated
- Compatible with Ubuntu 22.04 (most common student environment)

**Alternatives Considered**:
- ROS2 Iron/Rolling: Too bleeding-edge for educational content; API changes frequently
- ROS2 Foxy: Older LTS, approaching end-of-life (May 2023)

**Implementation Impact**: All code examples reference ROS2 Humble APIs. Frontmatter metadata includes `ros2_version: "humble"` tag.

---

### Decision: Conceptual Code Snippets (10-30 lines)

**Rationale**:
- Students focus on understanding patterns, not debugging environment setup
- Reduces cognitive load (spec: FR-007)
- Faster lesson completion (<45 min per spec SC-001)
- Enables learning without local ROS2 installation

**Alternatives Considered**:
- Full executable tutorials: Too time-consuming, requires complex setup, shifts focus from concepts to troubleshooting
- Pseudocode only: Too abstract, doesn't leverage students' Python knowledge

**Implementation Impact**: Code examples are syntactically correct Python with type hints, but omit boilerplate (e.g., full node lifecycle, error handling). Comments explain what each section does conceptually.

---

### Decision: Emoji Callout System (💬 🎓 🤝)

**Rationale**:
- Visual engagement improves retention (pedagogical research)
- Breaks up text-heavy content
- Signals different types of interaction (AI prompt, expert insight, practice exercise)
- Consistent with reference book structure (github.com/panaversity/ai-native-software-development)

**Callout Types**:
- **💬 AI Colearning Prompt**: Questions students can explore with Claude/ChatGPT to deepen understanding
- **🎓 Expert Insight**: Advanced perspectives, common pitfalls, real-world context
- **🤝 Practice Exercise**: Hands-on tasks (conceptual or guided experimentation)

**Implementation Impact**: Each lesson includes 2-3 callouts distributed throughout content. Technical Reviewer Agent validates that prompts are appropriate for student level.

---

## Lesson Content Structure Pattern

### Decision: 7-Section Template

**Structure**:
1. **# Main Title** - Clear, student-facing lesson name
2. **## What Is [Concept]?** - Definition and context
3. **## Why [Concept] Matters for Physical AI** - Real-world relevance, motivation
4. **### Key Principles** - Core ideas (3-5 enumerated points)
5. **## Practical Example** - Code snippet with explanation
6. **## Summary** - Key takeaways (3-5 bullets)
7. **## Next Steps** - Preview of next lesson

**Rationale**:
- Follows "What → Why → How → Takeaways" learning progression
- Consistent structure reduces cognitive load
- Clear semantic chunking (H2/H3) enables RAG retrieval (Principle I)
- Matches reference book pattern

**Implementation Impact**: All 4 lessons use this template. Outline Agent generates sections 1-7 as skeleton. Content Agent fills each section.

---

## Frontmatter Metadata Schema

### Decision: Extended YAML Schema (13 fields)

**Required Fields**:
- `title`: Lesson name
- `sidebar_position`: Numeric order for Docusaurus navigation
- `skills`: Array of skill objects (name, proficiency_level, category, bloom_level, digcomp_area, measurable_at_this_level)
- `learning_objectives`: Array of objective objects (objective, proficiency_level, bloom_level, assessment_method)
- `cognitive_load`: Object (new_concepts count, assessment string)
- `differentiation`: Object (extension_for_advanced, remedial_for_struggling)
- `tags`: Array of keyword strings
- `generated_by`: "agent" or "manual"
- `created`: ISO date
- `last_modified`: ISO date
- `ros2_version`: "humble" (Module 1 specific)

**Rationale**:
- Rich metadata enables Phase 2 RAG retrieval (Principle I)
- Bloom's taxonomy levels enable measurable learning objectives (spec FR-015)
- Differentiation supports varied learning paces (spec: edge case handling)
- Skills taxonomy prepares for personalization (Phase 2)

**Implementation Impact**: Frontmatter Agent generates compliant YAML for every lesson. Validation script checks schema correctness.

---

## ROS2 Concept Coverage (Module 1)

### Decision: 4-Lesson Progression

**Lesson 1: ROS2 Fundamentals**
- What ROS2 is (middleware, distributed systems)
- Why robotics needs ROS2 (modularity, real-time, hardware abstraction)
- Core architecture (nodes, graph, master-less design)
- Pub/sub model (conceptual)

**Lesson 2: Nodes, Topics, and Services**
- Node definition and lifecycle
- Topics (asynchronous pub/sub)
- Services (synchronous request/response)
- When to use each pattern (data streams vs commands)

**Lesson 3: Python rclpy Bridge**
- rclpy library overview
- Creating a simple node (class structure)
- Publishing messages (publisher setup, timer callbacks)
- Subscribing to topics (callback functions, message handling)

**Lesson 4: URDF for Humanoid Robots**
- What URDF is (robot description format)
- Links (rigid body parts: torso, arms, legs)
- Joints (connections: revolute, prismatic, fixed)
- Kinematic tree (parent-child relationships)
- Humanoid example (simplified arm structure)

**Rationale**: Progressive complexity (understanding → application → integration). Each lesson builds on previous concepts. Matches user stories P1-P4 in spec.md.

---

## Agent Pipeline Implementation Strategy

### Decision: Manual → Template → Agent (YAGNI)

**Phase 1 (Lesson 1)**: Manual authoring
- Author creates all content manually following templates
- Documents time spent on each section
- Identifies repetitive tasks
- Produces "golden example" for pattern matching

**Phase 2 (Lessons 2-3)**: Template refinement
- Copy-paste-modify approach using Lesson 1 as template
- Refine content structure based on learnings
- Begin drafting agent prompts for repetitive sections

**Phase 3 (Lesson 4)**: Agent creation
- If pattern stabilizes and task repeats ≥3 times, build agent
- Start with simple agents (Outline Agent, Frontmatter Agent)
- Hold off on complex agents (Content Agent) until pattern proven

**Rationale**: Constitution Principle VII (Minimal & Iterative Agent Development). Avoids premature abstraction. Validates approach before investing in automation.

**Implementation Impact**: tasks.md will show manual tasks for Lesson 1, semi-automated for Lessons 2-3, agent-driven for Lesson 4.

---

## Technical Accuracy Validation

### Decision: Two-Stage Review Process

**Stage 1: Automated Checks**
- Python syntax validation (code examples parse correctly)
- YAML schema validation (frontmatter matches spec)
- Markdown linting (headings, links, formatting)

**Stage 2: Technical Reviewer Agent**
- ROS2 concept accuracy (nodes, topics, services definitions match official docs)
- rclpy API correctness (method names, parameters, patterns)
- URDF syntax validation (XML structure, required attributes)
- Pedagogical appropriateness (explanations match student level)

**Rationale**: Constitution Principle V (Quality Gate: Technical Accuracy). Automated checks catch syntax errors fast. Agent review catches conceptual errors and pedagogical mismatches.

**Implementation Impact**: Technical Reviewer Agent contract defines review checklist. Agent runs after Content Agent, before Structure & Style Agent.

---

## Image and Diagram Strategy

### Decision: 4 Core Diagrams

**Required Diagrams**:
1. **ROS2 Architecture Diagram** (Lesson 1): Shows nodes, topics, DDS layer, applications
2. **Pub/Sub Flow Diagram** (Lesson 2): Message flow from publisher → topic → subscriber
3. **Node Graph Example** (Lesson 2): Multiple nodes communicating via topics/services
4. **URDF Humanoid Structure** (Lesson 4): Simplified humanoid arm with links/joints labeled

**Format**: PNG or SVG, <200KB per image (constitution performance target)

**Rationale**: Visual learners benefit from diagrams. Complex distributed systems concepts (ROS2 graph) are hard to explain in text alone.

**Implementation Impact**: Diagrams stored in `static/img/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/`. Created manually or using draw.io/excalidraw.

---

## Capstone Project Design

### Decision: Conceptual Design Exercise (Not Full Implementation)

**Project**: "Design a Simple Delivery Robot System"

**Requirements**:
- Student designs ROS2 node architecture (which nodes, what topics/services)
- Writes pseudocode or conceptual rclpy snippets (not full working code)
- Describes URDF structure for simple wheeled robot (base, wheels, sensors)
- Explains design decisions (why topic vs service, node responsibilities)

**Rationale**: Matches "conceptual understanding" goal (spec). Students apply all 4 lesson concepts without needing local ROS2 installation. Reduces time burden (<2 hours vs full implementation).

**Implementation Impact**: Capstone project file (`05-capstone-project.md`) includes design prompts, example solutions, evaluation rubric.

---

## Quiz Assessment Approach

### Decision: Mixed Format (15 questions)

**Question Types**:
- Multiple choice (8): Test factual knowledge (What is a ROS2 node? When to use a service vs topic?)
- Short answer (5): Test conceptual understanding (Explain pub/sub model. Describe URDF purpose.)
- Code reading (2): Test rclpy comprehension (Identify what code snippet does, trace message flow)

**Passing Criteria**: 80% correct (12/15) aligns with spec SC-001

**Rationale**: Mixed format assesses different Bloom's levels (remember, understand, apply). Automated grading for MC, manual review for short answer.

**Implementation Impact**: Quiz file (`06-quiz.md`) includes questions, answer key, and grading rubric.

---

## Summary of Research Findings

**Technical Decisions**:
- ROS2 Humble as reference distribution
- Conceptual code snippets (10-30 lines, type-hinted Python)
- 7-section lesson template (What → Why → Principles → Example → Summary → Next)
- Extended YAML frontmatter (13 fields for RAG-readiness)

**Pedagogical Decisions**:
- Emoji callout system (💬 🎓 🤝) for engagement
- 4-lesson progression (fundamentals → nodes → rclpy → URDF)
- Conceptual capstone project (design exercise, not full implementation)
- Mixed-format quiz (15 questions, 80% passing)

**Process Decisions**:
- Manual → Template → Agent progression (YAGNI)
- Two-stage validation (automated syntax + agent review)
- 4 core diagrams for visual learning

**All unknowns from Technical Context resolved. Ready for Phase 1 (Design).**
