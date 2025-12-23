---
description: "Task list for Module 4: Vision-Language-Action (VLA)"
---

# Tasks: Module 4 - Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/004-vision-language-action/`
**Prerequisites**: plan.md, spec.md, data-model.md, quickstart.md, contracts/

**Tests**: NOT INCLUDED - This feature focuses on educational content creation, not software testing. Validation occurs through Technical Reviewer Agent and manual user review.

**Organization**: Tasks are grouped by user story (lesson) to enable independent implementation and testing of each educational unit.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (lesson) this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project with content in `book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize module directory structure and foundational assets

- [x] T001 [P] Create module directory structure at book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/
- [x] T002 [P] Create images directory at book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/
- [x] T003 [P] Create module overview README at book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/README.md
- [x] T004 [P] Load and validate Module 4 lesson template from specs/004-vision-language-action/contracts/lesson-structure.md
- [x] T005 [P] Load and validate frontmatter schema from specs/004-vision-language-action/contracts/frontmatter-schema.yaml

**Checkpoint**: Module structure ready - lesson content creation can begin after foundational phase

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Establish templates, validation tools, and Technical Reviewer Agent before lesson creation

**⚠️ CRITICAL**: No lesson content can be created until this phase is complete

- [x] T006 Create placeholder SVG diagrams for VLA architecture in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/vla-architecture.svg
- [x] T007 [P] Create placeholder SVG diagrams for Whisper integration in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/whisper-integration.svg
- [x] T008 [P] Create placeholder SVG diagrams for LLM task decomposition in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/llm-task-decomposition.svg
- [x] T009 [P] Create placeholder SVG diagrams for multimodal fusion in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/multimodal-fusion.svg
- [x] T010 [P] Create placeholder SVG diagrams for action execution loop in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/action-execution-loop.svg
- [x] T011 [P] Create placeholder SVG diagrams for capstone system in book-source/static/img/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/autonomous-humanoid-capstone.svg
- [x] T012 Implement Technical Reviewer Agent validation workflow for VLA concepts, Whisper, LLMs, and action execution per contracts/technical-reviewer-agent.md
- [x] T013 [P] Create markdown validation script (heading hierarchy, frontmatter schema, word count) for Module 4
- [x] T014 [P] Verify local Docusaurus development environment works (npm start) with existing modules

**Checkpoint**: Foundation ready - lesson content creation can now begin in parallel

---

## Phase 3: User Story 1 - Voice-to-Action Systems (Priority: P1) 🎯 MVP

**Goal**: Teach students how voice commands are processed and converted into robotic actions, including speech recognition with OpenAI Whisper, natural language understanding, and command execution in ROS2

**Independent Test**: Student can explain the voice command pipeline from speech to action, describe how Whisper processes audio, and understand the challenges of natural language processing for robotics. Measured via quiz questions 1-4.

**Files**: `01-voice-to-action.md`, `01-voice-to-action.summary.md`

### Implementation for User Story 1 (Lesson 1)

- [x] T015 [US1] Create lesson outline following 7-section template (What Is → Why Matters → Key Principles → Practical Example → Summary → Next Steps) in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/01-voice-to-action.md
- [x] T016 [US1] Write "What Is Voice-to-Action?" section (400-500 words): definition, speech recognition for robotics, Whisper API, audio preprocessing
- [x] T017 [US1] Write "Why Voice-to-Action Matters for Humanoid Robots" section (400-500 words): intuitive interface, accessibility, human-robot interaction
- [x] T018 [US1] Write "Key Principles" subsection with 3-5 core voice-to-action concepts (audio preprocessing, speech recognition, command parsing, confidence scoring, error handling)
- [x] T019 [US1] Add 💬 AI Colearning Prompt callout: "Ask Claude to explain how Whisper processes audio for robotics applications"
- [x] T020 [US1] Add 🎓 Expert Insight callout: Speech recognition challenges in robotic environments (noise, accents, real-time processing)
- [x] T021 [US1] Write "Practical Example" section (300-400 words): conceptual Whisper integration workflow for humanoid robot voice commands
- [x] T022 [US1] Add 🤝 Practice Exercise callout: "Design a voice command processing pipeline for a humanoid robot cleaning task"
- [x] T023 [US1] Write "Summary" section (200-300 words): 3-5 key takeaways
- [x] T024 [US1] Write "Next Steps" section (100-150 words): preview of Lesson 2 (Cognitive Planning with LLMs)
- [x] T025 [US1] Create frontmatter metadata YAML with all 13 fields (skills, learning_objectives, cognitive_load, tags, etc.) at top of 01-voice-to-action.md
- [x] T026 [US1] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/01-voice-to-action.summary.md
- [x] T027 [US1] Run Technical Reviewer Agent validation with focus_areas: ["whisper", "voice_recognition", "pedagogy"]
- [x] T028 [US1] Fix any critical/major issues identified by Technical Reviewer Agent
- [x] T029 [US1] Run markdown validation (word count 1500-2500, heading hierarchy, frontmatter schema)
- [x] T030 [US1] Test Docusaurus rendering locally (npm start, verify frontmatter, navigation, formatting)

**Checkpoint**: Lesson 1 complete and validated - establishes "golden example" pattern for subsequent lessons

---

## Phase 4: User Story 2 - Cognitive Planning with LLMs (Priority: P2)

**Goal**: Teach students how Large Language Models (LLMs) can be used to translate high-level natural language commands into detailed sequences of robotic actions, including task decomposition, planning, and execution

**Independent Test**: Student can explain how LLMs decompose high-level commands into action sequences, describe the challenges of grounding language in physical reality, and understand the integration with ROS2 action servers. Measured via quiz questions 5-8.

**Files**: `02-cognitive-planning.md`, `02-cognitive-planning.summary.md`

### Implementation for User Story 2 (Lesson 2)

- [x] T031 [US2] Create lesson outline by duplicating Lesson 1 structure and updating topic to "Cognitive Planning with LLMs" in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/02-cognitive-planning.md
- [x] T032 [US2] Write "What Is Cognitive Planning?" section (400-500 words): LLMs for robotics, task decomposition, natural language understanding, grounding in physical reality
- [x] T033 [US2] Write "Why Cognitive Planning Matters" section (400-500 words): bridging human intent and robot execution, autonomous behavior, complex task execution
- [x] T034 [US2] Write "Key Principles" subsection covering LLM prompting (CoT, few-shot), task decomposition (hierarchical planning), language grounding (connecting words to actions), error handling (clarification requests)
- [x] T035 [US2] Add 💬 AI Colearning Prompt callout: "Ask Claude to demonstrate how 'Clean the room' gets decomposed into robotic actions"
- [x] T036 [US2] Add 🎓 Expert Insight callout: Limitations of LLMs for robotics (hallucinations, reasoning errors, safety considerations)
- [x] T037 [US2] Write "Practical Example" section (300-400 words): LLM-based task planning for humanoid robot cleaning task
- [x] T038 [US2] Add 🤝 Practice Exercise callout: "Analyze a complex command and break it down into ROS2 action sequences"
- [x] T039 [US2] Write "Summary" section (200-300 words): LLM capabilities, prompting strategies, grounding challenges
- [x] T040 [US2] Write "Next Steps" section (100-150 words): preview of Lesson 3 (Vision-Language Integration)
- [x] T041 [US2] Create frontmatter metadata YAML with updated skills (LLMs, Task Decomposition, Natural Language Grounding) and learning objectives in 02-cognitive-planning.md
- [x] T042 [US2] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/02-cognitive-planning.summary.md
- [x] T043 [US2] Run Technical Reviewer Agent validation with focus_areas: ["llm_planning", "task_decomposition", "pedagogy"]
- [x] T044 [US2] Fix any critical/major issues identified by Technical Reviewer Agent
- [x] T045 [US2] Run markdown validation and test Docusaurus rendering

**Checkpoint**: Lesson 2 complete - students understand cognitive planning with LLMs

---

## Phase 5: User Story 3 - Vision-Language Integration (Priority: P3)

**Goal**: Teach students how computer vision and language models work together to create Vision-Language-Action systems, enabling robots to understand and interact with their environment using both visual and linguistic information

**Independent Test**: Student can explain how vision and language models integrate, describe the challenges of object grounding, and understand multimodal processing for robotics. Measured via quiz questions 9-12.

**Files**: `03-vision-language-integration.md`, `03-vision-language-integration.summary.md`

### Implementation for User Story 3 (Lesson 3)

- [x] T046 [US3] Create lesson outline by duplicating Lesson 1 structure and updating topic to "Vision-Language Integration" in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/03-vision-language-integration.md
- [x] T047 [US3] Write "What Is Vision-Language Integration?" section (400-500 words): multimodal models, CLIP/BLIP, object detection with language context, visual grounding
- [x] T048 [US3] Write "Why Vision-Language Integration Matters" section (400-500 words): embodied AI, object identification, language-grounded perception, multimodal understanding
- [x] T049 [US3] Write "Key Principles" subsection covering multimodal fusion (feature combination, attention mechanisms), object grounding (connecting language to visual entities), reference resolution (handling "the red cup on the left"), vision-language models (CLIP, BLIP, etc.)
- [x] T050 [US3] Add 💬 AI Colearning Prompt callout: "Ask Claude to explain how 'the blue book' gets grounded in a visual scene"
- [x] T051 [US3] Add 🎓 Expert Insight callout: Challenges of multimodal fusion in robotics environments (occlusions, lighting, novel objects)
- [x] T052 [US3] Write "Practical Example" section (300-400 words): Vision-language system for identifying and manipulating objects based on natural language commands
- [x] T053 [US3] Add 🤝 Practice Exercise callout: "Design a vision-language system for identifying objects mentioned in voice commands"
- [x] T054 [US3] Write "Summary" section (200-300 words): multimodal fusion, grounding concepts, vision-language models
- [x] T055 [US3] Write "Next Steps" section (100-150 words): preview of Lesson 4 (Action Execution and Control)
- [x] T056 [US3] Create frontmatter metadata YAML with updated skills (Vision-Language Models, Multimodal Fusion, Object Grounding) and learning objectives in 03-vision-language-integration.md
- [x] T057 [US3] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/03-vision-language-integration.summary.md
- [x] T058 [US3] Run Technical Reviewer Agent validation with focus_areas: ["vision_language", "multimodal_fusion", "pedagogy"]
- [x] T059 [US3] Fix any critical/major issues identified by Technical Reviewer Agent
- [x] T060 [US3] Run markdown validation and test Docusaurus rendering

**Checkpoint**: Lesson 3 complete - students understand vision-language integration

---

## Phase 6: User Story 4 - Action Execution and Control (Priority: P4)

**Goal**: Integrate knowledge from all three lessons to design complete VLA systems that execute planned actions on robotic platforms, including ROS2 action servers, manipulation control, navigation, and error handling in the VLA pipeline

**Independent Test**: Student can explain the complete VLA pipeline, describe how action servers execute tasks, and understand the feedback loops between perception, planning, and execution. Measured via quiz questions 13-15 and capstone project.

**Files**: `04-action-execution-control.md`, `04-action-execution-control.summary.md`

### Implementation for User Story 4 (Lesson 4)

- [x] T061 [US4] Create lesson outline by duplicating Lesson 1 structure and updating topic to "Action Execution and Control" in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/04-action-execution-control.md
- [x] T062 [US4] Write "What Is VLA Action Execution?" section (400-500 words): complete pipeline from voice to action, ROS2 action servers, feedback control loops
- [x] T063 [US4] Write "Why Action Execution Matters" section (400-500 words): bridging planning and physical behavior, safety considerations, error handling
- [x] T064 [US4] Write "Key Principles" subsection covering action servers (navigation, manipulation), feedback loops (perception-planning-action), error handling (recovery strategies), safety mechanisms (validation of LLM-generated plans)
- [x] T065 [US4] Add 💬 AI Colearning Prompt callout: "Ask Claude to diagram the complete VLA pipeline from voice command to robot action"
- [x] T066 [US4] Add 🎓 Expert Insight callout: Safety considerations in autonomous VLA systems and validation of LLM-generated actions
- [x] T067 [US4] Write "Practical Example" section (300-400 words): Complete VLA system executing a cleaning task from voice command to final action
- [x] T068 [US4] Add 🤝 Practice Exercise callout: "Identify potential failure points in the VLA pipeline and propose mitigation strategies"
- [x] T069 [US4] Write "Summary" section (200-300 words): complete VLA pipeline, action execution, safety and error handling
- [x] T070 [US4] Write "Next Steps" section (100-150 words): preview of capstone project (complete autonomous humanoid VLA task)
- [x] T071 [US4] Create frontmatter metadata YAML with updated skills (Action Execution, ROS2 Action Servers, Safety Validation) and learning objectives in 04-action-execution-control.md
- [x] T072 [US4] Create summary file with 3-5 bullet points (100-200 words) in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/04-action-execution-control.summary.md
- [x] T073 [US4] Run Technical Reviewer Agent validation with focus_areas: ["action_execution", "safety", "pedagogy"]
- [x] T074 [US4] Fix any critical/major issues identified by Technical Reviewer Agent
- [x] T075 [US4] Run markdown validation and test Docusaurus rendering

**Checkpoint**: All 4 lessons complete - students have complete VLA knowledge for capstone

---

## Phase 7: Capstone Project (Integrative Exercise)

**Goal**: Create hands-on project integrating all 4 lesson concepts (Voice-to-Action, Cognitive Planning, Vision-Language Integration, Action Execution)

**Files**: `05-capstone-project.md`, `05-capstone-project.summary.md`

- [x] T076 Design capstone project concept: "The Autonomous Humanoid - Complete VLA Task" integrating voice recognition, cognitive planning, vision-language processing, and action execution
- [x] T077 Write project description (300-400 words) explaining the complete VLA scenario and what students will design in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/05-capstone-project.md
- [x] T078 Create requirements list (5-7 items): system architecture diagram, voice command processing, LLM task decomposition, vision-language integration, action execution plan
- [x] T079 Write guidance section (400-500 words) with hints, example structure, and design prompts
- [x] T080 Create evaluation rubric with 4-5 criteria (System Architecture 25pts, Voice Processing 20pts, Cognitive Planning 20pts, Vision-Language Integration 20pts, Action Execution 15pts)
- [x] T081 Add setup instructions for optional hands-on execution (API key guidance, simulation environment setup)
- [x] T082 Create frontmatter metadata YAML with skills from all 4 lessons and assessment_method: "capstone project" in 05-capstone-project.md
- [x] T083 Create summary file (100-150 words) describing capstone goals in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/05-capstone-project.summary.md
- [x] T084 Run markdown validation and test Docusaurus rendering

**Checkpoint**: Capstone complete - students have integrative project to demonstrate understanding

---

## Phase 8: Quiz (Assessment)

**Goal**: Create 15-20 question quiz assessing comprehension across all 4 lessons (80% passing = 12/15 correct)

**Files**: `06-quiz.md`

- [x] T085 Create quiz structure with 15-20 questions in book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/06-quiz.md
- [x] T086 Write questions 1-4 covering Lesson 1 (Voice-to-Action): 2 multiple choice, 2 short answer
- [x] T087 Write questions 5-8 covering Lesson 2 (Cognitive Planning): 3 multiple choice, 1 short answer
- [x] T088 Write questions 9-12 covering Lesson 3 (Vision-Language Integration): 2 multiple choice, 1 short answer, 1 scenario-based question
- [x] T089 Write questions 13-15 covering Lesson 4 (Action Execution): 1 multiple choice, 1 system architecture question, 1 safety/troubleshooting scenario
- [x] T090 Write questions 16-18 as integrative questions combining multiple lessons: 1 short answer, 1 scenario-based multiple choice, 1 system design question
- [x] T091 Create answer key with correct answers and explanations for all questions
- [x] T092 Write grading rubric: multiple choice (1pt each), short answer (1-2pts with partial credit), scenario-based (2-3pts with rubric)
- [x] T093 Add instructions: passing score 12/15 (80%), reference specific lessons for review
- [x] T094 Run Technical Reviewer Agent validation with focus_areas: ["whisper", "llm_planning", "vision_language", "action_execution", "pedagogy"]
- [x] T095 Fix any issues identified by Technical Reviewer Agent
- [x] T096 Test quiz questions with sample student profile (ensure appropriate difficulty for students with Modules 1-3 knowledge)

**Checkpoint**: Quiz complete - assessment tool ready for measuring student comprehension

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Finalize module with navigation, deployment preparation, and final validation

- [x] T097 [P] Update module README with navigation links to all 4 lessons + capstone + quiz at book-source/docs/13-Physical-AI-Humanoid-Robotics/04-vision-language-action/README.md
- [x] T098 [P] Update chapter README with Module 4 description and link at book-source/docs/13-Physical-AI-Humanoid-Robotics/README.md
- [x] T099 [P] Verify all sidebar_position values are correct (1-4 for lessons, 5 for capstone, 6 for quiz) in frontmatter
- [x] T100 [P] Ensure all 6 SVG diagrams are properly referenced in lesson content and render correctly
- [x] T101 Run full Docusaurus build (npm run build) and fix any build errors
- [x] T102 Validate all 13 files exist and follow naming conventions (01-04 lessons with .summary pairs, 05 capstone with .summary, 06 quiz, README)
- [x] T103 Run cross-lesson consistency check: terminology, formatting, frontmatter schema
- [x] T104 Verify all learning objectives from spec.md are covered across the 4 lessons
- [x] T105 Verify all success criteria from spec.md are measurable (quiz questions map to SC-001, capstone maps to SC-002-004)
- [x] T106 User review and final approval of all module content
- [x] T107 Prepare deployment: ensure GitHub Pages configuration supports new module
- [x] T108 Commit all changes with message "docs(module-04): complete Module 4 - Vision-Language-Action (VLA)"

**Checkpoint**: Module 4 complete and ready for deployment to GitHub Pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson content creation
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - Lessons can proceed sequentially (recommended for Module 4: establish pattern in Lesson 1, refine in 2-3, automate in 4)
  - Or in parallel if multiple authors available (not recommended for first module of this complexity)
- **Capstone (Phase 7)**: Depends on all 4 lessons being complete
- **Quiz (Phase 8)**: Depends on all 4 lessons being complete (can run in parallel with Phase 7)
- **Polish (Phase 9)**: Depends on Phases 3-8 completion

### User Story Dependencies

- **User Story 1 (Lesson 1 - Voice-to-Action)**: Can start after Foundational (Phase 2) - No dependencies on other lessons. ESTABLISHES PATTERN for subsequent lessons.
- **User Story 2 (Lesson 2 - Cognitive Planning)**: Can start after Foundational (Phase 2) - Conceptually builds on Lesson 1 but independently testable. REFINES PATTERN from Lesson 1.
- **User Story 3 (Lesson 3 - Vision-Language Integration)**: Can start after Foundational (Phase 2) - Conceptually builds on Lessons 1-2 but independently testable. REFINES PATTERN further.
- **User Story 4 (Lesson 4 - Action Execution)**: Can start after Foundational (Phase 2) - Independent topic, can use Lesson 1-3 as reference for pattern. OPTIONALLY AUTOMATES PATTERN with agents.

### Within Each Lesson

**Recommended Sequential Order**:
1. Create outline (T015, T031, T046, T061)
2. Write main sections (What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps)
3. Add callouts (AI Colearning, Expert Insight, Practice Exercise)
4. Create frontmatter metadata
5. Create summary file
6. Run Technical Reviewer Agent validation
7. Fix issues
8. Final validation and rendering test

**Parallelizable within lesson**: Main content sections can be written in parallel by different authors, then integrated.

### Parallel Opportunities

- **Phase 1 (Setup)**: Tasks T001-T005 marked [P] can run in parallel
- **Phase 2 (Foundational)**: Tasks T007-T011, T013-T014 marked [P] can run in parallel
- **SVG Creation**: Tasks T006-T011 can run in parallel after T002 completes
- **Lessons 2-4**: Can start in parallel AFTER Lesson 1 establishes pattern (not recommended for quality, but possible for speed)
- **Phase 7 & 8**: Capstone and Quiz can be developed in parallel once all 4 lessons complete
- **Phase 9 (Polish)**: Tasks T097-T100 marked [P] can run in parallel

---

## Parallel Example: Lesson 1 Main Sections

```bash
# These content sections can be written in parallel by different authors:
Task T016: "Write 'What Is Voice-to-Action?' section"
Task T017: "Write 'Why Voice-to-Action Matters' section"
Task T021: "Write 'Practical Example' section"
Task T023: "Write 'Summary' section"

# Then integrate sequentially:
Task T025: "Create frontmatter metadata"
Task T027: "Run Technical Reviewer Agent"
```

---

## Implementation Strategy

### MVP First (Lesson 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T014) - CRITICAL, establishes validation framework
3. Complete Phase 3: User Story 1 / Lesson 1 (T015-T030)
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
7. Deploy to GitHub Pages → Module 4 live!

### Sequential vs Parallel Strategy

**Recommended for Module 4: SEQUENTIAL**
- Lesson 1 establishes "golden example" (manual, ~6 hours)
- Lessons 2-3 copy-paste-modify pattern (~4 hours each)
- Lesson 4 optionally uses agents (~3 hours)
- Ensures consistency and quality

**Parallel Strategy (Not Recommended for Module 4)**
- Multiple authors work on Lessons 1-4 simultaneously
- Risk of inconsistent patterns, formatting
- Requires strong style guide and coordination
- Consider for future modules after pattern established

---

## Task Summary

**Total Tasks**: 108
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 8 tasks
- Phase 3 (Lesson 1): 16 tasks
- Phase 4 (Lesson 2): 13 tasks
- Phase 5 (Lesson 3): 13 tasks
- Phase 6 (Lesson 4): 14 tasks
- Phase 7 (Capstone): 9 tasks
- Phase 8 (Quiz): 12 tasks
- Phase 9 (Polish): 12 tasks

**Parallel Opportunities Identified**: 25 tasks marked [P]

**Independent Test Criteria per Story**:
- **US1 (Lesson 1)**: Student can explain voice-to-action pipeline conceptually, identify components, describe Whisper integration
- **US2 (Lesson 2)**: Student can describe LLM cognitive planning and explain task decomposition for robotics
- **US3 (Lesson 3)**: Student can explain vision-language integration and multimodal processing for robotics
- **US4 (Lesson 4)**: Student can design complete VLA system integrating all components with safety considerations

**Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (Lesson 1 only) = 29 tasks

---

## Format Validation

✅ **ALL tasks follow checklist format**: `- [ ] [TaskID] [P?] [Story?] Description with file path`
✅ **Task IDs sequential**: T001 → T108
✅ **Story labels present**: [US1], [US2], [US3], [US4] for all lesson tasks (Phases 3-6)
✅ **Parallel markers present**: [P] for 25 tasks that can run independently
✅ **File paths explicit**: All content creation tasks specify exact file locations

---

## Notes

- Tests are NOT included - this is educational content, not software. Validation occurs via Technical Reviewer Agent and user review.
- Each lesson (User Story) is independently completable and testable via quiz questions and capstone project.
- Technical Reviewer Agent is MANDATORY for all lessons (Constitution Principle V).
- Commit after each lesson completion, not after each task (avoids excessive commits).
- Stop at any checkpoint to validate lesson quality independently.
- Docusaurus rendering tests ensure GitHub Pages deployment readiness.
- Content must acknowledge that full API access (Whisper, LLMs) is optional but provide conceptual understanding for students without API access (per spec assumptions).