---

description: "Task list for Module 1: The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-nervous-system/`
**Prerequisites**: plan.md, spec.md, data-model.md, quickstart.md, contracts/

**Tests**: NOT INCLUDED - This feature focuses on educational content creation, not software testing. Validation occurs through Technical Reviewer Agent and manual user review.

**Organization**: Tasks are grouped by user story (lesson) to enable independent implementation and testing of each educational unit.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (lesson) this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project with content in `book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and module structure

- [X] T001 Initialize Docusaurus project in book-source/ with TypeScript config
- [X] T002 [P] Create module directory structure at book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/
- [X] T003 [P] Create images directory at book-source/static/img/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/
- [X] T004 [P] Configure sidebars.ts to include Module 1 navigation
- [X] T005 [P] Create chapter overview README at book-source/docs/13-Physical-AI-Humanoid-Robotics/README.md
- [X] T006 Create module overview README at book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/README.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Establish templates, validation tools, and Technical Reviewer Agent before lesson creation

**⚠️ CRITICAL**: No lesson content can be created until this phase is complete

- [X] T007 Create lesson content template document based on 7-section pattern from research.md
- [X] T008 [P] Create frontmatter metadata template with all 13 required YAML fields from data-model.md
- [X] T009 [P] Document code snippet guidelines (10-30 lines, type hints, conceptual focus)
- [X] T010 [P] Document callout patterns (💬 AI Colearning, 🎓 Expert Insight, 🤝 Practice Exercise)
- [X] T011 Implement Technical Reviewer Agent validation workflow per contracts/technical-reviewer-agent.md
- [X] T012 [P] Create markdown validation script (heading hierarchy, frontmatter schema, word count)
- [X] T013 [P] Setup local Docusaurus development environment (npm install, npm start)

**Checkpoint**: Foundation ready - lesson content creation can now begin in parallel

---

## Phase 3: User Story 1 - ROS2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Teach students what ROS2 is, why it's essential for robotics, and how it serves as the "nervous system" for humanoid robots

**Independent Test**: Student can explain what ROS2 is, identify its core components (nodes, topics, graph), and describe the pub/sub model conceptually. Measured via quiz questions 1-4.

**Files**: `01-ros2-fundamentals.md`, `01-ros2-fundamentals.summary.md`

### Implementation for User Story 1 (Lesson 1)

- [X] T014 [US1] Create lesson outline following 7-section template (What Is → Why Matters → Key Principles → Practical Example → Summary → Next Steps) in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.md
- [X] T015 [US1] Write "What Is ROS2?" section (400-500 words): definition, middleware concept, distributed systems architecture
- [X] T016 [US1] Write "Why ROS2 Matters for Physical AI" section (400-500 words): modularity, real-time communication, hardware abstraction
- [X] T017 [US1] Write "Key Principles" subsection with 3-5 core ROS2 concepts (nodes, topics, graph visualization)
- [X] T018 [US1] Add 💬 AI Colearning Prompt callout: "Ask Claude to explain pub/sub model using real-world analogy"
- [X] T019 [US1] Add 🎓 Expert Insight callout: ROS2 vs ROS1 differences (master-less architecture, DDS)
- [X] T020 [US1] Write "Practical Example" section (300-400 words): conceptual ROS2 system for humanoid robot
- [X] T021 [US1] Add 🤝 Practice Exercise callout: "Diagram a simple robot system with 3 nodes and 2 topics"
- [X] T022 [US1] Write "Summary" section (200-300 words): 3-5 key takeaways
- [X] T023 [US1] Write "Next Steps" section (100-150 words): preview of Lesson 2 (node communication patterns)
- [X] T024 [US1] Create frontmatter metadata YAML with all 13 fields (skills, learning_objectives, cognitive_load, tags, etc.) at top of 01-ros2-fundamentals.md
- [X] T025 [US1] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.summary.md
- [X] T026 [US1] Run Technical Reviewer Agent validation with focus_areas: ["ros2_concepts", "pedagogy"]
- [X] T027 [US1] Fix any critical/major issues identified by Technical Reviewer Agent
- [X] T028 [US1] Run markdown validation (word count 1500-2500, heading hierarchy, frontmatter schema)
- [X] T029 [US1] Test Docusaurus rendering locally (npm start, verify frontmatter, navigation, formatting)

**Checkpoint**: Lesson 1 complete and validated - establishes "golden example" pattern for subsequent lessons

---

## Phase 4: User Story 2 - Node Communication (Priority: P2)

**Goal**: Teach students how different robot parts communicate using ROS2 nodes, topics (pub/sub), and services (request/response) to coordinate complex behaviors

**Independent Test**: Student can describe the difference between topics and services, and explain when to use each pattern. Measured via quiz questions 5-8.

**Files**: `02-nodes-topics-services.md`, `02-nodes-topics-services.summary.md`

### Implementation for User Story 2 (Lesson 2)

- [X] T030 [US2] Create lesson outline by duplicating Lesson 1 structure and updating topic to "Nodes, Topics, and Services" in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/02-nodes-topics-services.md
- [X] T031 [US2] Write "What Are Nodes?" section (400-500 words): independent processes, modularity, node lifecycle
- [X] T032 [US2] Write "Why Node Communication Matters" section (400-500 words): sensor-actuator coordination, distributed control
- [X] T033 [US2] Write "Key Principles" subsection covering topics (pub/sub, asynchronous, many-to-many) and services (request/response, synchronous, one-to-one)
- [X] T034 [US2] Add 💬 AI Colearning Prompt callout: "Ask Claude when to use topics vs services for a robot arm control scenario"
- [X] T035 [US2] Add 🎓 Expert Insight callout: QoS (Quality of Service) policies and their importance
- [X] T036 [US2] Write "Practical Example" section (300-400 words): humanoid robot sensor network (cameras, IMU publishing data)
- [X] T037 [US2] Add 🤝 Practice Exercise callout: "Design communication pattern for a delivery robot: sensors → navigation → motors"
- [X] T038 [US2] Write "Summary" section (200-300 words): when to use topics vs services, key communication patterns
- [X] T039 [US2] Write "Next Steps" section (100-150 words): preview of Lesson 3 (Python rclpy implementation)
- [X] T040 [US2] Create frontmatter metadata YAML with updated skills (Node Architecture, Topic Design, Service Patterns) and learning objectives in 02-nodes-topics-services.md
- [X] T041 [US2] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/02-nodes-topics-services.summary.md
- [X] T042 [US2] Run Technical Reviewer Agent validation with focus_areas: ["ros2_concepts", "pedagogy"]
- [X] T043 [US2] Fix any critical/major issues identified by Technical Reviewer Agent
- [X] T044 [US2] Run markdown validation and test Docusaurus rendering

**Checkpoint**: Lesson 2 complete - students understand node communication patterns conceptually

---

## Phase 5: User Story 3 - Python rclpy Bridge (Priority: P3)

**Goal**: Teach students how to connect their Python programming knowledge to ROS2 using rclpy library to create nodes, publish messages, and call services

**Independent Test**: Student can read and understand Python rclpy code, identify node initialization, publisher/subscriber creation, and explain what each line does conceptually. Measured via quiz questions 9-11 (including code reading questions).

**Files**: `03-python-rclpy-bridge.md`, `03-python-rclpy-bridge.summary.md`

### Implementation for User Story 3 (Lesson 3)

- [X] T045 [US3] Create lesson outline by duplicating Lesson 1 structure and updating topic to "Python rclpy Bridge" in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/03-python-rclpy-bridge.md
- [X] T046 [US3] Write "What Is rclpy?" section (400-500 words): Python client library for ROS2, bridging Python to ROS2 middleware
- [X] T047 [US3] Write "Why rclpy Matters" section (400-500 words): leveraging Python for robot control, AI integration, rapid prototyping
- [X] T048 [US3] Write "Key Principles" subsection covering rclpy.init(), Node class, create_publisher(), create_subscription(), create_timer()
- [X] T049 [US3] Create Python code example 1 (15-25 lines): Simple publisher node with type hints and clear comments in "Key Principles" section
- [X] T050 [US3] Create Python code example 2 (15-25 lines): Simple subscriber node with callback pattern in "Key Principles" section
- [X] T051 [US3] Add 💬 AI Colearning Prompt callout: "Ask Claude to explain the callback pattern in ROS2 subscribers and why it's asynchronous"
- [X] T052 [US3] Add 🎓 Expert Insight callout: Type hints in rclpy code and Python 3.11+ best practices
- [X] T053 [US3] Write "Practical Example" section (300-400 words): Humanoid robot status publisher (operational, battery level)
- [X] T054 [US3] Create Python code example 3 (20-30 lines): Combined publisher-subscriber node for robot status in "Practical Example" section
- [X] T055 [US3] Add 🤝 Practice Exercise callout: "Modify the example to add a new topic for robot joint positions"
- [X] T056 [US3] Write "Summary" section (200-300 words): rclpy node patterns, publisher/subscriber lifecycle
- [X] T057 [US3] Write "Next Steps" section (100-150 words): preview of Lesson 4 (URDF robot descriptions)
- [X] T058 [US3] Create frontmatter metadata YAML with updated skills (rclpy API, Python-ROS2 Integration) and learning objectives in 03-python-rclpy-bridge.md
- [X] T059 [US3] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/03-python-rclpy-bridge.summary.md
- [X] T060 [US3] Run Technical Reviewer Agent validation with focus_areas: ["ros2_concepts", "rclpy_api", "pedagogy"]
- [X] T061 [US3] Fix any critical/major issues identified by Technical Reviewer Agent
- [X] T062 [US3] Validate all Python code snippets parse correctly (python -m py_compile)
- [X] T063 [US3] Run markdown validation and test Docusaurus rendering

**Checkpoint**: Lesson 3 complete - students can read and understand rclpy code conceptually

---

## Phase 6: User Story 4 - URDF for Humanoid Robots (Priority: P4)

**Goal**: Teach students how robots are described structurally using URDF (Unified Robot Description Format) to define humanoid robot bodies, joints, and links for simulation and control

**Independent Test**: Student can read a simple URDF file and identify key components (links, joints, parent-child relationships). Student can explain what URDF represents conceptually. Measured via quiz questions 12-13.

**Files**: `04-urdf-humanoid-basics.md`, `04-urdf-humanoid-basics.summary.md`

### Implementation for User Story 4 (Lesson 4)

- [X] T064 [US4] Create lesson outline by duplicating Lesson 1 structure and updating topic to "URDF for Humanoid Robots" in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/04-urdf-humanoid-basics.md
- [X] T065 [US4] Write "What Is URDF?" section (400-500 words): XML format, robot description, kinematic trees
- [X] T066 [US4] Write "Why URDF Matters" section (400-500 words): simulation (Gazebo), motion planning, visualization (RViz)
- [X] T067 [US4] Write "Key Principles" subsection covering links (body parts), joints (connections, types), parent-child relationships
- [X] T068 [US4] Create URDF code example 1 (15-20 lines): Simple 2-link robot arm with XML syntax in "Key Principles" section
- [X] T069 [US4] Create URDF code example 2 (20-30 lines): Humanoid robot torso with shoulder and elbow joints in "Key Principles" section
- [X] T070 [US4] Add 💬 AI Colearning Prompt callout: "Ask Claude to explain the kinematic tree for a humanoid robot and how URDF represents it"
- [X] T071 [US4] Add 🎓 Expert Insight callout: URDF units (meters, radians) and coordinate frame conventions
- [X] T072 [US4] Write "Practical Example" section (300-400 words): Humanoid robot arm description with 3 joints (shoulder, elbow, wrist)
- [X] T073 [US4] Add 🤝 Practice Exercise callout: "Sketch a URDF structure for a humanoid robot leg with hip, knee, and ankle joints"
- [X] T074 [US4] Write "Summary" section (200-300 words): URDF structure, link/joint relationships, use cases
- [X] T075 [US4] Write "Next Steps" section (100-150 words): preview of capstone project (integrate all concepts)
- [X] T076 [US4] Create frontmatter metadata YAML with updated skills (URDF Syntax, Robot Modeling) and learning objectives in 04-urdf-humanoid-basics.md
- [X] T077 [US4] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/04-urdf-humanoid-basics.summary.md
- [X] T078 [US4] Run Technical Reviewer Agent validation with focus_areas: ["urdf_syntax", "pedagogy"]
- [X] T079 [US4] Fix any critical/major issues identified by Technical Reviewer Agent
- [X] T080 [US4] Validate all URDF XML snippets are syntactically valid
- [X] T081 [US4] Run markdown validation and test Docusaurus rendering

**Checkpoint**: All 4 lessons complete - students have foundational ROS2 knowledge for capstone

---

## Phase 7: Capstone Project (Integrative Exercise)

**Goal**: Create hands-on project integrating all 4 lesson concepts (ROS2, nodes/topics, rclpy, URDF)

**Files**: `05-capstone-project.md`, `05-capstone-project.summary.md`

- [X] T082 Design capstone project concept: "Design a Simple Delivery Robot System" integrating ROS2 architecture, node communication, rclpy code structure, and URDF robot model
- [X] T083 Write project description (300-400 words) explaining the delivery robot scenario and what students will design in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/05-capstone-project.md
- [X] T084 Create requirements list (5-7 items): node diagram, topic design, rclpy code outline, URDF robot structure
- [X] T085 Write guidance section (400-500 words) with hints, example structure, and design prompts
- [X] T086 Create evaluation rubric with 4-5 criteria (Node Architecture 25pts, Communication Design 25pts, rclpy Structure 25pts, URDF Model 25pts)
- [X] T087 Add setup instructions for optional hands-on execution (ROS2 installation guidance, point to tutorials)
- [X] T088 Create frontmatter metadata YAML with skills from all 4 lessons and assessment_method: "capstone project" in 05-capstone-project.md
- [X] T089 Create summary file (100-150 words) describing capstone goals in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/05-capstone-project.summary.md
- [X] T090 Run markdown validation and test Docusaurus rendering

**Checkpoint**: Capstone complete - students have integrative project to demonstrate understanding

---

## Phase 8: Quiz (Assessment)

**Goal**: Create 15-question quiz assessing comprehension across all 4 lessons (80% passing = 12/15 correct)

**Files**: `06-quiz.md`

- [X] T091 Create quiz structure with 15 questions in book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/06-quiz.md
- [X] T092 Write questions 1-4 covering Lesson 1 (ROS2 fundamentals): 2 multiple choice, 2 short answer
- [X] T093 Write questions 5-8 covering Lesson 2 (nodes/topics/services): 3 multiple choice, 1 short answer
- [X] T094 Write questions 9-11 covering Lesson 3 (rclpy): 2 multiple choice, 1 code reading question (read Python snippet, explain what it does)
- [X] T095 Write questions 12-13 covering Lesson 4 (URDF): 1 multiple choice, 1 code reading question (read URDF snippet, identify joints)
- [X] T096 Write questions 14-15 as integrative questions combining multiple lessons: 1 short answer, 1 scenario-based multiple choice
- [X] T097 Create answer key with correct answers and explanations for all 15 questions
- [X] T098 Write grading rubric: multiple choice (1pt each), short answer (1-2pts with partial credit), code reading (2pts with rubric)
- [X] T099 Add instructions: passing score 12/15 (80%), reference specific lessons for review
- [X] T100 Run Technical Reviewer Agent validation with focus_areas: ["ros2_concepts", "rclpy_api", "urdf_syntax", "pedagogy"]
- [X] T101 Fix any issues identified by Technical Reviewer Agent
- [X] T102 Test quiz questions with sample student profile (ensure appropriate difficulty for beginners)

**Checkpoint**: Quiz complete - assessment tool ready for measuring student comprehension

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Finalize module with navigation, deployment preparation, and final validation

- [X] T103 [P] Update module README with navigation links to all 4 lessons + capstone + quiz at book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/README.md
- [X] T104 [P] Update chapter README with Module 1 description and link at book-source/docs/13-Physical-AI-Humanoid-Robotics/README.md
- [X] T105 [P] Verify all sidebar_position values are correct (1-4 for lessons, 5 for capstone, 6 for quiz) in frontmatter
- [X] T106 [P] Add placeholder images for future diagrams in book-source/static/img/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/ (ros2-architecture.svg, pub-sub-diagram.svg, node-graph-example.svg, urdf-humanoid-structure.svg)
- [X] T107 Run full Docusaurus build (npm run build) and fix any build errors
- [X] T108 Validate all 11 files exist and follow naming conventions (01-04 lessons with .summary pairs, 05 capstone with .summary, 06 quiz, README)
- [X] T109 Run cross-lesson consistency check: terminology, formatting, frontmatter schema
- [X] T110 Verify all learning objectives from spec.md are covered across the 4 lessons
- [X] T111 Verify all success criteria from spec.md are measurable (quiz questions map to SC-001, capstone maps to SC-002-004)
- [ ] T112 User review and final approval of all module content
- [X] T113 Prepare deployment: configure GitHub Pages in docusaurus.config.ts
- [ ] T114 Commit all changes with message "docs(module-01): complete Module 1 - The Robotic Nervous System (ROS 2)"

**Checkpoint**: Module 1 complete and ready for deployment to GitHub Pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson content creation
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - Lessons can proceed sequentially (recommended for Module 1: establish pattern in Lesson 1, refine in 2-3, automate in 4)
  - Or in parallel if multiple authors available (not recommended for first module)
- **Capstone (Phase 7)**: Depends on all 4 lessons being complete
- **Quiz (Phase 8)**: Depends on all 4 lessons being complete (can run in parallel with Phase 7)
- **Polish (Phase 9)**: Depends on Phases 3-8 completion

### User Story Dependencies

- **User Story 1 (Lesson 1 - ROS2 Fundamentals)**: Can start after Foundational (Phase 2) - No dependencies on other lessons. ESTABLISHES PATTERN for subsequent lessons.
- **User Story 2 (Lesson 2 - Node Communication)**: Can start after Foundational (Phase 2) - Conceptually builds on Lesson 1 but independently testable. REFINES PATTERN from Lesson 1.
- **User Story 3 (Lesson 3 - Python rclpy)**: Can start after Foundational (Phase 2) - Conceptually builds on Lessons 1-2 but independently testable. REFINES PATTERN further.
- **User Story 4 (Lesson 4 - URDF)**: Can start after Foundational (Phase 2) - Independent topic, can use Lesson 1-3 as reference for pattern. OPTIONALLY AUTOMATES PATTERN with agents.

### Within Each Lesson

**Recommended Sequential Order**:
1. Create outline (T014, T030, T045, T064)
2. Write main sections (What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps)
3. Add callouts (AI Colearning, Expert Insight, Practice Exercise)
4. Create code examples (for Lessons 3-4)
5. Create frontmatter metadata
6. Create summary file
7. Run Technical Reviewer Agent validation
8. Fix issues
9. Final validation and rendering test

**Parallelizable within lesson**: Main content sections can be written in parallel by different authors, then integrated.

### Parallel Opportunities

- **Phase 1 (Setup)**: Tasks T002-T005 marked [P] can run in parallel
- **Phase 2 (Foundational)**: Tasks T008-T010, T012-T013 marked [P] can run in parallel
- **Lessons 2-4**: Can start in parallel AFTER Lesson 1 establishes pattern (not recommended for quality, but possible for speed)
- **Phase 7 & 8**: Capstone and Quiz can be developed in parallel once all 4 lessons complete
- **Phase 9 (Polish)**: Tasks T103-T106 marked [P] can run in parallel

---

## Parallel Example: Lesson 1 Main Sections

```bash
# These content sections can be written in parallel by different authors:
Task T015: "Write 'What Is ROS2?' section"
Task T016: "Write 'Why ROS2 Matters' section"
Task T020: "Write 'Practical Example' section"
Task T022: "Write 'Summary' section"

# Then integrate sequentially:
Task T024: "Create frontmatter metadata"
Task T026: "Run Technical Reviewer Agent"
```

---

## Implementation Strategy

### MVP First (Lesson 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T013) - CRITICAL, establishes validation framework
3. Complete Phase 3: User Story 1 / Lesson 1 (T014-T029)
4. **STOP and VALIDATE**: Review Lesson 1 as "golden example" pattern
5. User approval before proceeding to Lessons 2-4

**Rationale**: Lesson 1 is the pattern template for all subsequent lessons. Perfect it first.

### Incremental Delivery

1. Complete Setup + Foundational → Validation framework ready
2. Add Lesson 1 → Validate independently → Establish pattern ✅
3. Add Lesson 2 → Validate independently → Refine pattern ✅
4. Add Lesson 3 → Validate independently → Refine pattern ✅
5. Add Lesson 4 → Validate independently → Optionally automate ✅
6. Add Capstone + Quiz → Module complete ✅
7. Deploy to GitHub Pages → Module 1 live!

### Sequential vs Parallel Strategy

**Recommended for Module 1: SEQUENTIAL**
- Lesson 1 establishes "golden example" (manual, ~6 hours)
- Lessons 2-3 copy-paste-modify pattern (~4 hours each)
- Lesson 4 optionally uses agents (~3 hours)
- Ensures consistency and quality

**Parallel Strategy (Not Recommended for Module 1)**
- Multiple authors work on Lessons 1-4 simultaneously
- Risk of inconsistent patterns, formatting
- Requires strong style guide and coordination
- Consider for Modules 2-4 after pattern established

---

## Task Summary

**Total Tasks**: 114
- Phase 1 (Setup): 6 tasks
- Phase 2 (Foundational): 7 tasks
- Phase 3 (Lesson 1): 16 tasks
- Phase 4 (Lesson 2): 15 tasks
- Phase 5 (Lesson 3): 19 tasks
- Phase 6 (Lesson 4): 18 tasks
- Phase 7 (Capstone): 9 tasks
- Phase 8 (Quiz): 12 tasks
- Phase 9 (Polish): 12 tasks

**Parallel Opportunities Identified**: 18 tasks marked [P]

**Independent Test Criteria per Story**:
- **US1 (Lesson 1)**: Student can explain ROS2 conceptually, identify components, describe pub/sub model
- **US2 (Lesson 2)**: Student can describe topics vs services and explain when to use each
- **US3 (Lesson 3)**: Student can read rclpy code and explain what each line does conceptually
- **US4 (Lesson 4)**: Student can read URDF file and identify links, joints, parent-child relationships

**Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (Lesson 1 only) = 29 tasks

---

## Format Validation

✅ **ALL tasks follow checklist format**: `- [ ] [TaskID] [P?] [Story?] Description with file path`
✅ **Task IDs sequential**: T001 → T114
✅ **Story labels present**: [US1], [US2], [US3], [US4] for all lesson tasks (Phases 3-6)
✅ **Parallel markers present**: [P] for 18 tasks that can run independently
✅ **File paths explicit**: All content creation tasks specify exact file locations

---

## Notes

- Tests are NOT included - this is educational content, not software. Validation occurs via Technical Reviewer Agent and user review.
- Each lesson (User Story) is independently completable and testable via quiz questions and capstone project.
- Manual → Template → Agent progression follows YAGNI principle (Constitution Principle VII).
- Technical Reviewer Agent is MANDATORY for all lessons (Constitution Principle V).
- Commit after each lesson completion, not after each task (avoids excessive commits).
- Stop at any checkpoint to validate lesson quality independently.
- Docusaurus rendering tests ensure GitHub Pages deployment readiness.
