<!--
Sync Impact Report:
- Version change: [New Constitution] → 1.0.0
- Rationale: Initial constitution for Phase 1 (Book Writing)
- Added sections: Core Principles (7), Book Structure Standards, Subagent Architecture, Development Workflow, Quality Standards
- Templates requiring updates:
  ✅ plan-template.md - Constitution Check section aligns with principles
  ✅ spec-template.md - User stories support book chapter planning
  ✅ tasks-template.md - Task structure supports chapter pipeline
- Follow-up TODOs: Update constitution to v2.0.0 after Phase 1 completion (add chatbot, personalization, auth)
-->

# Physical AI & Humanoid Robotics Book Constitution (v1.0 - Book Writing Phase)

## Core Principles

### I. Content-First, RAG-Ready Structure (NON-NEGOTIABLE)

Every chapter MUST be written with future RAG (Retrieval-Augmented Generation) integration in mind:

- **Clear semantic chunking**: Use consistent heading hierarchy (H2 for sections, H3 for subsections)
- **Rich frontmatter metadata**: Every lesson includes `title`, `skills`, `learning_objectives`, `cognitive_load`, `tags`
- **Modular content blocks**: Sections can be shown/hidden independently for personalization
- **Summary pairs**: Each main lesson file MUST have a corresponding `.summary.md` file
- **Self-contained explanations**: No dangling references; each section explains concepts fully

**Rationale**: Phase 2 will add chatbot and personalization. Content structure determines RAG quality. Refactoring content later is expensive and error-prone.

### II. Spec-Driven Chapter Development

No content is written without a specification:

- **User defines learning goals** → Engineer drafts chapter spec → User approves
- Each chapter spec MUST include:
  - Learning objectives (measurable)
  - Target audience assumptions (CS students with Python knowledge)
  - Key concepts to cover
  - Code example requirements (conceptual snippets, not full tutorials)
  - Success criteria (what student should be able to do after)
- **Chapter pipeline executes only after spec approval**

**Rationale**: Prevents scope creep, ensures consistency, enables measurable outcomes.

### III. Subagent Pipeline Architecture

Content creation follows a **9-step agent pipeline** for every chapter:

**Pipeline Flow**:
```
Spec (User Approved)
  ↓
[1] Outline Agent → structured subsections
  ↓
[2] Chapter Content Agent → full MDX content
  ↓
[3] Case Study Generator → real-world robotics examples
  ↓
[4] Code Example Agent → ROS2/Python/URDF snippets
  ↓
[5] Technical Reviewer Agent → validates accuracy (ROS2, Gazebo, URDF concepts)
  ↓
[6] Structure & Style Agent → enforces heading hierarchy, frontmatter, summaries
  ↓
[7] Frontmatter Agent → generates compliant YAML metadata
  ↓
[8] Docusaurus Integration Agent → converts to final MDX
  ↓
[9] Review Checkpoint → User approval before commit
```

**Agent Contracts** (MANDATORY):
- **Input schema**: What the agent needs (e.g., topic, learning goals, outline)
- **Output schema**: What it produces (e.g., structured markdown, JSON metadata)
- **Acceptance criteria**: How to validate success
- **Failure modes**: What can go wrong, retry strategy

**Implementation**: Claude Code Subagents (using Task tool with custom agent prompts)

**Stateless Requirement**: Agents have no memory between invocations; all context passed via inputs.

**Composability**: Agents can be reused in Phase 2 for chatbot response validation, content updates, personalization.

**Rationale**: Systematic quality, parallel execution, reusable for Phase 2, demonstrates multi-agent reasoning (hackathon bonus points).

### IV. Book Structure Standards

**Repository Organization** (Monorepo):
```
book-source/
├── docs/
│   └── 13-Physical-AI-Humanoid-Robotics/
│       ├── README.md                          (Chapter overview)
│       ├── 01-ros2-nervous-system/            (Module 1 - 4 lessons)
│       │   ├── README.md                      (Module overview)
│       │   ├── 01-ros2-fundamentals.md
│       │   ├── 01-ros2-fundamentals.summary.md
│       │   ├── 02-nodes-topics-services.md
│       │   ├── 02-nodes-topics-services.summary.md
│       │   ├── 03-python-rclpy-bridge.md
│       │   ├── 03-python-rclpy-bridge.summary.md
│       │   ├── 04-urdf-humanoid-basics.md
│       │   ├── 04-urdf-humanoid-basics.summary.md
│       │   ├── 05-capstone-project.md
│       │   ├── 05-capstone-project.summary.md
│       │   └── 06-quiz.md
│       ├── 02-digital-twin-simulation/         (Module 2 - 4 lessons)
│       ├── 03-nvidia-isaac-ai-brain/          (Module 3 - 4 lessons)
│       └── 04-vision-language-action/         (Module 4 - 4 lessons)
├── src/                                        (Docusaurus React components)
├── static/                                     (Images, assets)
├── docusaurus.config.ts
├── sidebars.ts
└── package.json
```

**Naming Conventions**:
- Chapter: `##-Topic-In-Title-Case` (e.g., `13-Physical-AI-Humanoid-Robotics`)
- Modules: `##-module-name-kebab-case` (e.g., `01-ros2-nervous-system`)
- Lessons: `##-lesson-topic.md` + `##-lesson-topic.summary.md`
- Capstone always numbered `05-capstone-project.md`
- Quiz always numbered `06-quiz.md`

**Content Scope**:
- **4 modules** (ROS2, Simulation, Isaac, VLA)
- **4 lessons per module** (main content + capstone + quiz = 6 files per lesson)
- **16 total lessons** (4 × 4)
- **Audience**: CS students with Python knowledge
- **Code depth**: Conceptual snippets (not full working tutorials)

### V. Quality Gate: Technical Accuracy (NON-NEGOTIABLE)

All technical content MUST pass Technical Reviewer Agent before commit:

**Review Scope**:
- ROS2 concepts (nodes, topics, services, parameters)
- URDF syntax and semantics
- Gazebo physics simulation accuracy
- Python/rclpy code correctness
- NVIDIA Isaac Sim workflows
- Navigation (Nav2) patterns

**Failure Response**: Agent retries with corrections; human review only if agent cannot resolve.

**Rationale**: Technical errors in educational content erode trust. Automated review catches errors before user sees them.

### VI. Incremental Commits & Feature Branches

**Git Workflow**:
- **Feature branch per chapter**: `feature/module-##-lesson-##`
- **Commit after each pipeline step**: e.g., "Add outline for ROS2 fundamentals", "Add technical review for URDF lesson"
- **Commit message format**: `docs(module-##): <imperative verb> <what changed>`
  - Examples:
    - `docs(module-01): add outline for ROS2 fundamentals`
    - `docs(module-02): add code examples for Gazebo physics`
    - `docs(module-01): fix technical review feedback on URDF syntax`
- **Merge to main**: After user approves final lesson (all 9 pipeline steps complete)
- **No direct commits to main**

**Rationale**: Traceable progress, rollback safety, clear history for debugging.

### VII. Minimal & Iterative Agent Development

**Agents are NOT created upfront. They are built when**:
- A task repeats ≥3 times
- A format becomes standardized
- A workflow becomes obvious

**First Iteration**: Manual work to establish pattern
**Second Iteration**: Template-based
**Third Iteration**: Build agent

**Avoids**: Premature abstraction, over-engineering, unused agents

**Rationale**: YAGNI principle. Focus on delivering content first, optimize second.

## Book Structure Standards

### Frontmatter Schema (MANDATORY)

Every lesson file MUST include:

```yaml
---
title: "Lesson Title"
sidebar_position: ##
skills:
  - name: "ROS2 Fundamentals"
    proficiency_level: "beginner"
    category: "robotics-middleware"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "explain pub/sub model"
learning_objectives:
  - objective: "Understand ROS2 nodes and topics"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz + capstone project"
cognitive_load:
  new_concepts: 5
  assessment: "moderate - builds on Python knowledge"
differentiation:
  extension_for_advanced: "Explore ROS2 services and actions"
  remedial_for_struggling: "Review Python basics first"
tags: ["ros2", "robotics", "middleware"]
generated_by: "agent"
created: "YYYY-MM-DD"
last_modified: "YYYY-MM-DD"
---
```

### Content Structure Pattern

```markdown
# Main Title

## What Is [Concept]?
[Clear definition, context]

## Why [Concept] Matters for Physical AI
[Real-world relevance, motivation]

### Key Principles
[Core ideas, enumerated]

### 💬 AI Colearning Prompt
> Prompt students to explore with Claude/ChatGPT

### 🎓 Expert Insight
> Advanced perspective or common pitfall

## Practical Example
[Code snippet with explanation]

### 🤝 Practice Exercise
> Hands-on task for students

## Summary
[Key takeaways, 3-5 bullets]

## Next Steps
[What comes next in the module]
```

### Code Example Standards

- **Language**: Python 3.11+ (type hints mandatory)
- **Length**: 10-30 lines (conceptual, not production)
- **Format**: Fenced code blocks with syntax highlighting
- **Explanation**: Line-by-line or block-level comments
- **Execution**: Not required to run (conceptual snippets)

Example:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """Publishes messages to a ROS2 topic."""

    def __init__(self) -> None:
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self) -> None:
        msg = String()
        msg.data = 'Robot operational'
        self.publisher.publish(msg)
```

## Subagent Architecture

### Agent Taxonomy

**A) Authoring Agents**:
1. **Outline Agent**: Takes topic → outputs structured subsections
2. **Chapter Content Agent**: Takes outline → writes full MDX content
3. **Case Study Generator**: Creates real-world robotics examples
4. **Code Example Agent**: Generates Python/ROS2/URDF snippets

**B) Validation Agents**:
5. **Technical Reviewer Agent**: Validates ROS2/URDF/Gazebo accuracy
6. **Structure & Style Agent**: Enforces heading hierarchy, summaries, frontmatter

**C) Integration Agents**:
7. **Frontmatter Agent**: Generates compliant YAML metadata
8. **Docusaurus Integration Agent**: Converts to final MDX with sidebar config

### Agent Contract Template

Every agent MUST define:

```markdown
## Agent Name: [Name]

### Input Schema
- `topic`: string (required) - lesson topic
- `learning_goals`: array<string> (required) - what students should learn
- `context`: object (optional) - prior knowledge assumptions

### Output Schema
- `type`: "markdown" | "json" | "yaml"
- `structure`: [describe output format]

### Acceptance Criteria
- [ ] Output follows template structure
- [ ] No placeholder text (e.g., [TODO], [FILL])
- [ ] Meets length requirements (if applicable)
- [ ] Passes validation (syntax, schema)

### Failure Modes
- **Invalid topic**: Retry with clarification prompt
- **Missing context**: Request user input
- **Format violation**: Auto-fix with Structure & Style Agent
```

### Agent Reusability (Phase 2)

Same agents will be used for:
- **Chatbot answer validation**: Technical Reviewer validates RAG responses
- **Personalization**: Structure & Style Agent adapts content based on user profile
- **Content updates**: Chapter Content Agent regenerates sections with new information
- **Knowledge extraction**: Frontmatter Agent generates embeddings metadata

## Development Workflow

### Chapter Development Process

**Phase 0: Specification**
1. User defines learning goals for chapter
2. Engineer drafts chapter spec (learning objectives, key concepts, success criteria)
3. User reviews and approves spec

**Phase 1: Pipeline Execution**
1. Create feature branch: `feature/module-##-lesson-##`
2. Execute 9-step agent pipeline (Outline → Content → Case Study → Code → Review → Style → Frontmatter → Docusaurus → User Review)
3. Commit after each step
4. User approves final output

**Phase 2: Integration**
1. Merge feature branch to main
2. Deploy to GitHub Pages (auto via GitHub Actions)
3. Validate live deployment

**Phase 3: Iteration**
1. Collect feedback (if any)
2. Create new feature branch for fixes
3. Re-run affected pipeline steps
4. Merge and redeploy

### Review Checkpoints

**After Each Pipeline Step**:
- Agent completes task
- Output validated against acceptance criteria
- Commit to feature branch
- Proceed to next step

**Before Merge**:
- User reviews complete lesson (all files: main, summary, capstone, quiz)
- Validates against chapter spec
- Approves or requests changes

**After Deployment**:
- User validates live page on GitHub Pages
- Checks navigation, formatting, code blocks, images

## Quality Standards

### Priority Ranking (1 = Highest)

1. **Code Quality** (tests, maintainability, documentation)
2. **Performance** (fast page loads, efficient builds)
3. **Security** (no secrets in repo, safe dependencies)

### Code Quality Gates

**Documentation**:
- Every agent MUST have contract documentation
- Every module MUST have README with overview
- Every lesson MUST have summary file

**Maintainability**:
- DRY principle: Reuse agents, don't duplicate logic
- Clear naming: Descriptive file/folder names
- Consistent structure: Follow book structure standards

**Testing** (Phase 1 - Manual):
- User validates content accuracy
- Technical Reviewer Agent validates code/concepts
- No automated tests for content (yet)

### Performance Targets

**Build Time**: <2 minutes for full Docusaurus build
**Page Load**: <3 seconds for lesson page
**Image Optimization**: <200KB per image, WebP format preferred

### Security Requirements

**Secrets**: Never commit API keys, tokens, credentials
**Dependencies**: Use `npm audit` to check for vulnerabilities
**Access Control**: GitHub Pages is public (no sensitive content)

## Governance

### Constitution Authority

This constitution supersedes all other practices for Phase 1 (Book Writing).

**Scope**: Book content creation, agent architecture, development workflow
**Out of Scope**: Chatbot, personalization, authentication (Phase 2)

### Amendment Process

**Version Bump Rules**:
- **MAJOR (x.0.0)**: Backward-incompatible changes (e.g., remove pipeline step, change structure)
- **MINOR (1.x.0)**: New principles, new agents, expanded guidance
- **PATCH (1.0.x)**: Clarifications, wording fixes, typo corrections

**Amendment Steps**:
1. Propose change with rationale
2. User approval required
3. Update constitution file
4. Propagate to dependent templates (plan, spec, tasks)
5. Document in Sync Impact Report (HTML comment at top of file)
6. Create PHR for amendment

### Compliance Review

**Every PR MUST**:
- Follow git workflow (feature branch, commit messages)
- Pass Technical Reviewer Agent
- Include summary file for main lesson
- Have compliant frontmatter

**Agent Pipeline MUST**:
- Execute all 9 steps in order
- Validate against agent contracts
- Commit after each step

**Content MUST**:
- Match approved chapter spec
- Target CS students with Python knowledge
- Use conceptual code snippets (not full tutorials)
- Be RAG-ready (clear chunking, rich metadata)

### Phase 2 Transition

After Phase 1 completion (all 16 lessons deployed to GitHub Pages):

1. Review Phase 1 learnings
2. Draft v2.0.0 constitution including:
   - Chatbot architecture (FastAPI, Neon Postgres, Qdrant)
   - Personalization system (Better-Auth, user profiles)
   - Translation system (Urdu support)
   - Agent reuse strategy
3. User approves v2.0.0 constitution
4. Begin Phase 2 development

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
