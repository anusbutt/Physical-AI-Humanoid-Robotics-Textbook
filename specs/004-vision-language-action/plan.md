# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `004-vision-language-action` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-vision-language-action/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Module 4 builds on ROS2 fundamentals, sensor perception, and Isaac platform knowledge (Modules 1-3) to teach CS students the convergence of LLMs and robotics through Vision-Language-Action (VLA) systems. The module covers 4 comprehensive lessons: (1) Voice-to-Action systems using OpenAI Whisper, (2) Cognitive Planning with LLMs for natural language understanding, (3) Vision-Language integration for multimodal perception, and (4) Action Execution and Control for complete VLA pipeline implementation. Students will learn to build systems that process voice commands, decompose natural language into action sequences, integrate vision-language processing, and execute robotic actions. Deliverables include 4 lessons + capstone project + quiz, all following the established Docusaurus educational content structure with RAG-ready metadata.

## Technical Context

**Language/Version**: Python 3.11+ (type hints mandatory), Markdown/MDX (Docusaurus v3), OpenAI Whisper API integration, LLM APIs (OpenAI GPT, Claude), ROS2 Humble (conceptual reference)
**Primary Dependencies**: Docusaurus v3 (TypeScript), ROS2 Humble (conceptual reference), OpenAI Whisper API, LLM APIs, Isaac Sim for simulation (from Module 3), sensor_msgs for audio integration
**Storage**: Static markdown files in `book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/`
**Testing**: Manual content validation, Technical Reviewer Agent (VLA concepts, API integration, ROS2 action execution), Docusaurus build validation
**Target Platform**: GitHub Pages (static site deployment), browser-based learning
**Project Type**: Educational content (static site) - extends existing Docusaurus book structure
**Performance Goals**: <45 min lesson completion time, <3 sec page load, <2 min Docusaurus build
**Constraints**: Content must be conceptual (API access optional - provide mock examples for students without API keys), emphasize multimodal integration without requiring complex AI model training, assume Modules 1-3 completion
**Scale/Scope**: 4 lessons × ~2000 words each, 6 total files (4 lessons + capstone + quiz), 13-field frontmatter metadata per file, VLA architecture diagrams and API integration examples as optional downloads

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Content-First, RAG-Ready Structure (NON-NEGOTIABLE)
- **Compliant**: All lessons will use consistent H2/H3 hierarchy for semantic chunking
- **Compliant**: 13-field frontmatter metadata schema established in previous modules (skills, learning_objectives, cognitive_load, tags, etc.)
- **Compliant**: Each lesson file paired with `.summary.md` file
- **Compliant**: Self-contained explanations with embedded AI colearning prompts

### ✅ Spec-Driven Chapter Development
- **Compliant**: Spec approved (spec.md created with 4 user stories, 13 functional requirements, 8 success criteria)
- **Compliant**: Learning objectives measurable (FR-002, FR-003)
- **Compliant**: Target audience defined (CS students with Python + Modules 1-3 knowledge)
- **Compliant**: Success criteria measurable (SC-001: 45 min completion, 80% quiz accuracy)

### ✅ Subagent Pipeline Architecture
- **Compliant**: Will follow 9-step agent pipeline from constitution:
  1. Outline Agent → structured subsections
  2. Chapter Content Agent → full MDX content
  3. Case Study Generator → real-world VLA examples (Tesla Optimus, Figure robots, Amazon Astro)
  4. Code Example Agent → ROS2/Python/VLA API integration snippets
  5. Technical Reviewer Agent → validates VLA concepts, Whisper integration, LLM prompting, action execution
  6. Structure & Style Agent → enforces heading hierarchy, frontmatter, summaries
  7. Frontmatter Agent → generates compliant YAML metadata
  8. Docusaurus Integration Agent → converts to final MDX
  9. Review Checkpoint → User approval before commit

### ✅ Book Structure Standards
- **Compliant**: Module follows established naming: `04-vision-language-action/`
- **Compliant**: Lessons numbered 01-04, capstone 05, quiz 06
- **Compliant**: Each lesson has paired `.summary.md` file
- **Compliant**: Repository structure matches previous modules pattern

### ✅ Quality Gate: Technical Accuracy (NON-NEGOTIABLE)
- **Compliant**: Technical Reviewer Agent validates:
  - Voice-to-Action systems (OpenAI Whisper integration, audio preprocessing, speech recognition for robotics)
  - Cognitive Planning concepts (LLM prompting strategies, task decomposition, language grounding in ROS2)
  - Vision-Language integration (multimodal models, object detection, language-grounded perception)
  - Action Execution (ROS2 action servers, manipulation interfaces, feedback control loops)

### ✅ Incremental Commits & Feature Branches
- **Compliant**: Feature branch `004-vision-language-action` created
- **Compliant**: Commit message format: `docs(module-04): <verb> <what>`
- **Compliant**: Commit after each pipeline step
- **Compliant**: Merge to main after user approval

### ✅ Minimal & Iterative Agent Development
- **Compliant**: Agents already established in previous modules (reusing pipeline)
- **Compliant**: Templates standardized (lesson structure, frontmatter schema)
- **Compliant**: No new agents needed (reuse existing 9-step pipeline)

**Gate Status**: ✅ **PASSED** - All constitution principles satisfied. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/004-vision-language-action/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (lesson templates, frontmatter contracts)
│   ├── frontmatter-schema.yaml
│   ├── lesson-structure.md
│   └── agent-pipeline.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/
├── docs/
│   └── 13-Physical-AI-Humanoid-Robotics/
│       ├── README.md                                    (Chapter overview)
│       ├── 01-ros2-nervous-system/                      (Module 1 - COMPLETE)
│       ├── 02-sensors-perception/                       (Module 2 - COMPLETE)
│       ├── 03-isaac-ai-brain/                           (Module 3 - COMPLETE)
│       └── 04-vision-language-action/                   (Module 4 - THIS FEATURE)
│           ├── README.md                                (Module 4 overview)
│           ├── 01-voice-to-action.md                    (Lesson 1: OpenAI Whisper Integration)
│           ├── 01-voice-to-action.summary.md
│           ├── 02-cognitive-planning.md                 (Lesson 2: LLMs for Natural Language)
│           ├── 02-cognitive-planning.summary.md
│           ├── 03-vision-language-integration.md        (Lesson 3: Multimodal Perception)
│           ├── 03-vision-language-integration.summary.md
│           ├── 04-action-execution-control.md           (Lesson 4: Complete VLA Pipeline)
│           ├── 04-action-execution-control.summary.md
│           ├── 05-capstone-project.md                   (Autonomous Humanoid VLA Task)
│           ├── 05-capstone-project.summary.md
│           └── 06-quiz.md                               (Assessment)
├── static/
│   └── img/
│       └── 13-Physical-AI-Humanoid-Robotics/
│           └── 04-vision-language-action/
│               ├── vla-architecture.svg                 (Voice-Language-Action pipeline)
│               ├── whisper-integration.svg              (Speech recognition flow)
│               ├── llm-task-decomposition.svg           (Natural language to actions)
│               ├── multimodal-fusion.svg                (Vision-language processing)
│               ├── action-execution-loop.svg            (Perception-planning-action cycle)
│               └── autonomous-humanoid-capstone.svg     (Complete VLA system diagram)
├── docusaurus.config.ts                                 (Existing - no changes)
├── sidebars.ts                                          (Auto-generated from filesystem)
└── package.json                                         (Existing - no changes)
```

**Structure Decision**: Educational content structure follows established previous modules pattern. All markdown files in `book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/` with paired summaries. Images in `static/img/` with matching directory structure. No backend/API needed (static site). Reuses existing Docusaurus configuration from previous modules. Optional VLA configuration files and API integration examples provided as downloadable examples (not embedded in lessons).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All constitution gates passed.

---

## Phase 0: Research & Outline

### Research Questions

Based on Technical Context and spec requirements, the following areas need research to inform content creation:

1. **Voice-to-Action Systems with OpenAI Whisper**
   - How does OpenAI Whisper work for speech recognition and what are its capabilities for robotics applications?
   - What audio preprocessing is needed for optimal Whisper performance in robotic environments (noise reduction, format conversion)?
   - How do you integrate Whisper API with ROS2 systems for real-time voice command processing?
   - What are the latency and accuracy characteristics of Whisper for robotics applications?
   - How do you handle different languages, accents, and background noise in robotic environments?
   - What are the cost considerations and rate limits for Whisper API in continuous robotic applications?

2. **Cognitive Planning with LLMs**
   - How do LLMs (GPT, Claude) decompose high-level natural language commands into action sequences?
   - What prompting strategies work best for task decomposition in robotics contexts?
   - How do you ground natural language in physical reality (e.g., "clean the room" → "find trash" → "pick up trash" → "dispose of trash")?
   - What are the limitations of current LLMs for robotics task planning (hallucinations, reasoning errors)?
   - How do you handle ambiguous or underspecified commands with LLMs?
   - What are the best practices for error handling when LLMs generate incorrect action sequences?

3. **Vision-Language Integration**
   - How do multimodal models (CLIP, BLIP) enable vision-language integration for robotics?
   - What techniques exist for grounding language in visual scenes (object detection with language context)?
   - How do you handle object reference resolution (e.g., "the red cup on the left")?
   - What are the challenges of multimodal fusion for robotics perception?
   - How do vision-language models handle novel objects or scenes not in training data?
   - What are the computational requirements for real-time vision-language processing on robots?

4. **Action Execution and Control**
   - How do you integrate LLM-generated action sequences with ROS2 action servers?
   - What ROS2 action interfaces are needed for VLA systems (navigation, manipulation, perception)?
   - How do you handle action execution failures and provide feedback to the cognitive planner?
   - What feedback control loops exist between perception, planning, and execution in VLA systems?
   - How do you ensure safety when executing LLM-generated action sequences?
   - What are the latency requirements for real-time VLA pipeline execution?

5. **VLA System Integration**
   - How does the complete VLA pipeline work: voice → Whisper → LLM → vision-language → action execution?
   - What are the key integration challenges (synchronization, error propagation, feedback loops)?
   - How do you design the system architecture for the complete VLA pipeline?
   - What are the performance requirements for end-to-end VLA systems?
   - How do you handle failures at different stages of the VLA pipeline?

### Research Execution Plan

**Method**: Use Task tool with Explore agent to research each topic systematically.

**For each research question**:
1. Search OpenAI documentation (platform.openai.com/docs) for Whisper and LLM integration
2. Review robotics literature on VLA systems (recent papers, implementations)
3. Examine existing VLA implementations (Tesla Optimus, Figure robots, Amazon Astro, ALOHA)
4. Identify educational resources (VLA tutorials, multimodal robotics examples)
5. Review ROS2 integration patterns (audio input, action servers, multimodal perception)

**Output**: `research.md` documenting findings, design decisions, and rationale for lesson content structure.

---

## Phase 1: Design & Contracts

### Data Model

**Entities** (from spec Key Entities section):

1. **Lesson**
   - **Fields**: title, sidebar_position, skills[], learning_objectives[], cognitive_load, differentiation, tags[], created, last_modified, ros2_version
   - **Relationships**: Each lesson has 1 paired summary file, belongs to 1 module, references specific VLA concepts
   - **Validation**: 13-field frontmatter schema (established in previous modules)
   - **State**: Draft → Technical Review → Style Review → Approved → Published

2. **VLA Component** (Conceptual Entity for Lesson 1)
   - **Attributes**: Voice recognition module (Whisper), audio preprocessing pipeline, speech-to-text conversion, confidence scoring, language identification
   - **Purpose**: Converts human speech commands to text for downstream processing
   - **Representation in Content**: Architecture diagrams, API integration examples, latency/accuracy trade-off discussions
   - **Student Interaction**: Conceptual understanding of speech recognition; optional API examples with mock responses

3. **Cognitive Planner** (Conceptual Entity for Lesson 2)
   - **Attributes**: LLM model (GPT/Claude), prompting strategy, task decomposition algorithm, grounding mechanism, error handling
   - **Purpose**: Translates natural language commands into sequences of robotic actions
   - **Representation in Content**: Prompt engineering examples, task decomposition workflows, grounding illustrations
   - **Student Interaction**: Understanding of LLM prompting for robotics; example prompts for common commands

4. **Vision-Language Processor** (Conceptual Entity for Lesson 3)
   - **Attributes**: Multimodal model (CLIP/BLIP), object detection module, language grounding algorithm, feature fusion mechanism
   - **Purpose**: Integrates visual perception with language understanding for object identification and manipulation
   - **Representation in Content**: Multimodal architecture diagrams, object reference resolution examples, vision-language fusion concepts
   - **Student Interaction**: Understanding of multimodal processing; examples of language-grounded perception

5. **Action Executor** (Conceptual Entity for Lesson 4)
   - **Attributes**: ROS2 action servers (navigation, manipulation), feedback control loops, error handling, safety mechanisms
   - **Purpose**: Executes planned actions on robotic platforms with safety and error handling
   - **Representation in Content**: Action server interfaces, execution flow diagrams, error recovery strategies
   - **Student Interaction**: Understanding of ROS2 action execution; example action sequences for common tasks

6. **VLA Integration Challenge** (Design Pattern for Lesson 4)
   - **Attributes**: Synchronization requirements, error propagation, feedback loops, performance optimization, safety constraints
   - **Purpose**: Challenges specific to integrating all VLA components into a cohesive system
   - **Representation in Content**: System architecture diagrams, integration patterns, failure mode analyses
   - **Student Interaction**: Design complete VLA systems considering integration challenges

**Output**: `data-model.md` with expanded entity descriptions and relationships.

### Contracts

**Agent Input/Output Contracts** (following established previous modules pattern):

1. **Outline Agent**
   - **Input**: Lesson topic (e.g., "Voice-to-Action Systems"), learning goals from spec (Whisper integration, speech recognition), target lesson length (~2000 words)
   - **Output**: Structured outline with 7 sections (What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps), subsection titles, estimated word counts per section
   - **Acceptance**: Outline covers all learning objectives from spec, follows established 7-section pattern, includes callout placeholders (💬 AI Colearning, 🎓 Expert Insight, 🤝 Practice Exercise)

2. **Chapter Content Agent**
   - **Input**: Approved outline, research.md findings (Whisper capabilities, LLM prompting, multimodal fusion), spec user story acceptance scenarios
   - **Output**: Full MDX content (~2000 words) with H2/H3 hierarchy, embedded callouts, clear explanations building on previous modules knowledge
   - **Acceptance**: Content meets word count target, covers all acceptance scenarios from spec, uses consistent heading structure, explains concepts conceptually (no deep implementation details)

3. **Case Study Generator**
   - **Input**: Lesson topic, research.md real-world examples (Tesla Optimus, Figure robots, Amazon Astro)
   - **Output**: 1-2 case studies per lesson showing practical applications (e.g., "Figure robots use LLMs for task decomposition in warehouse environments")
   - **Acceptance**: Case studies are technically accurate, relevant to lesson concepts, demonstrate real-world impact of technologies

4. **Code Example Agent**
   - **Input**: Lesson topic, ROS2 integration requirements (Whisper API calls, LLM prompting, vision-language processing, action execution)
   - **Output**: 10-30 line code snippets with type hints, comments explaining key concepts, focus on conceptual understanding (not production code)
   - **Acceptance**: Code examples are syntactically correct, demonstrate key concepts, appropriate for CS students (no advanced AI model training or complex control theory)

5. **Technical Reviewer Agent**
   - **Input**: Complete lesson content (MDX), spec acceptance criteria, focus areas (Whisper integration, LLM prompting, vision-language fusion, action execution)
   - **Output**: Validation report with severity levels (critical/major/minor issues), specific line references for errors, suggested corrections
   - **Acceptance**: All critical and major issues identified; technical accuracy validated against OpenAI documentation and robotics best practices

6. **Structure & Style Agent**
   - **Input**: Technically reviewed content, constitution content structure pattern (7-section template), callout requirements (2× AI Colearning, 2× Expert Insight, 2× Practice Exercise)
   - **Output**: Content with enforced heading hierarchy (H2/H3 semantic chunking), embedded callouts at appropriate locations, summary section with 3-5 key takeaways
   - **Acceptance**: Structure matches constitution template, callouts strategically placed, summary concise and comprehensive

7. **Frontmatter Agent**
   - **Input**: Final lesson content, lesson number (01-04), learning objectives from spec, cognitive load assessment (new concepts count, difficulty level)
   - **Output**: 13-field YAML frontmatter (title, sidebar_position, skills, learning_objectives, cognitive_load, differentiation, tags, generated_by, created, last_modified, ros2_version)
   - **Acceptance**: All 13 fields populated, skills aligned with lesson content, learning objectives map to spec acceptance criteria, tags follow established vocabulary

8. **Docusaurus Integration Agent**
   - **Input**: Content with frontmatter, lesson number for filename (01-voice-to-action.md)
   - **Output**: Final MDX file + paired summary file (.summary.md with frontmatter + Summary section only)
   - **Acceptance**: File placed in correct directory (04-vision-language-action/), summary file created, Docusaurus rendering validated (no broken links, frontmatter parses correctly)

9. **Review Checkpoint**
   - **Input**: Complete lesson (main + summary files), user review request
   - **Output**: User approval or requested changes
   - **Acceptance**: User confirms lesson meets quality standards and aligns with learning goals

**Output**: `/contracts/agent-pipeline.md`, `/contracts/lesson-structure.md`, `/contracts/frontmatter-schema.yaml`

### Quickstart Guide

**Developer Onboarding** for Module 4 Content Creation:

1. **Prerequisites**:
   - Modules 1-3 content complete and deployed
   - Docusaurus development environment running (`npm start` in `book-source/`)
   - Access to OpenAI documentation (platform.openai.com/docs)
   - Access to VLA research papers and implementations
   - Technical Reviewer Agent configured (`.specify/templates/technical-reviewer-prompt.md`)

2. **Content Creation Workflow**:
   - Follow 9-step agent pipeline for each lesson (01-04)
   - Commit after each agent step: `git commit -m "docs(module-04): add outline for voice-to-action"`
   - Run Docusaurus build after each lesson to catch errors early: `cd book-source && npm run build`
   - User review after lesson complete before proceeding to next lesson

3. **Key Resources**:
   - Spec: `specs/004-vision-language-action/spec.md` (user stories, acceptance criteria)
   - Research: `specs/004-vision-language-action/research.md` (Whisper integration, LLM prompting, multimodal fusion)
   - Contracts: `specs/004-vision-language-action/contracts/` (agent input/output schemas)
   - Constitution: `.specify/memory/constitution.md` (content quality standards)
   - Module 1-3 Examples: `book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/`, `02-sensors-perception/`, `03-isaac-ai-brain/`

4. **Testing & Validation**:
   - Technical accuracy validated by Technical Reviewer Agent (focus: Whisper, LLMs, vision-language, action execution)
   - Structure validated by Structure & Style Agent (7-section pattern, callouts, summaries)
   - Docusaurus build validated: `cd book-source && npm run build` (must succeed with zero errors)

5. **Deployment**:
   - After all 6 files complete (4 lessons + capstone + quiz) and user approved, merge `004-vision-language-action` branch to main
   - GitHub Actions automatically deploys to GitHub Pages
   - Verify deployment: https://anusbutt.github.io/ros2-physical-ai-book/

**Output**: `quickstart.md` in specs directory.

---

## Post-Design Constitution Re-Check

*Verify all gates still pass after Phase 1 design*

### ✅ Content-First, RAG-Ready Structure
- **Compliant**: Data model includes rich frontmatter metadata (13 fields) for RAG retrieval
- **Compliant**: Content structure enforced via Structure & Style Agent (H2/H3 semantic chunking)
- **Compliant**: Summary pairs mandatory (Docusaurus Integration Agent creates both files)

### ✅ Spec-Driven Chapter Development
- **Compliant**: Contracts map to spec acceptance scenarios (agent input/output schemas align with user story requirements)
- **Compliant**: Learning objectives measurable via quiz and capstone project

### ✅ Subagent Pipeline Architecture
- **Compliant**: All 9 agents have defined input/output contracts in `/contracts/agent-pipeline.md`
- **Compliant**: Stateless agents (all context passed via inputs, no memory between invocations)
- **Compliant**: Composable agents (can be reused for content updates, chatbot validation in Phase 2)

### ✅ Book Structure Standards
- **Compliant**: Naming conventions followed (04-vision-language-action/, 01-voice-to-action.md, etc.)
- **Compliant**: 4 lessons + capstone + quiz structure matches previous modules

### ✅ Quality Gate: Technical Accuracy
- **Compliant**: Technical Reviewer Agent validates Whisper integration, LLM prompting, vision-language fusion, action execution
- **Compliant**: Focus areas defined in contracts (VLA components, API integration, ROS2 action execution)

### ✅ Incremental Commits & Feature Branches
- **Compliant**: Commit message format defined in quickstart.md (`docs(module-04): <verb> <what>`)
- **Compliant**: Commit after each pipeline step enforced in workflow

### ✅ Minimal & Iterative Agent Development
- **Compliant**: Reusing existing 9-step pipeline from previous modules (no new agents created)

**Post-Design Gate Status**: ✅ **PASSED** - All constitution principles satisfied after Phase 1 design.

---

## Summary of Phase 0 & Phase 1 Outputs

**Phase 0 Deliverables**:
- ✅ `research.md`: Research findings on Whisper, LLMs, vision-language integration, action execution, VLA systems

**Phase 1 Deliverables**:
- ✅ `data-model.md`: Lesson entity, VLA component, cognitive planner, vision-language processor, action executor, VLA integration challenges
- ✅ `contracts/agent-pipeline.md`: 9-agent input/output contracts
- ✅ `contracts/lesson-structure.md`: 7-section content pattern with callout requirements
- ✅ `contracts/frontmatter-schema.yaml`: 13-field metadata schema (established in previous modules)
- ✅ `quickstart.md`: Developer onboarding guide for Module 4 content creation

**Phase 2 (Not Executed by /sp.plan)**:
- `/sp.tasks` command generates `tasks.md` with detailed task breakdown (T001-T###) for implementing all 4 lessons + capstone + quiz

**Ready for `/sp.tasks`**: Yes - all design artifacts complete, constitution gates passed.