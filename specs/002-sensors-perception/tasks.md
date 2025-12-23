# Tasks: Module 2 - Sensors and Perception for Humanoid Robots

**Input**: Design documents from `/specs/002-sensors-perception/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: Not applicable for educational content creation (manual validation via Technical Reviewer Agent)

**Organization**: Tasks are grouped by user story (lesson) to enable independent implementation and testing of each lesson.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/lesson this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson files**: `book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/NN-lesson-name.md`
- **Summary files**: `book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/NN-lesson-name.summary.md`
- **Images**: `book-source/static/img/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/diagram-name.svg`
- **Module README**: `book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/README.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify Docusaurus structure and create Module 2 directory

- [X] T001 Create module directory structure at book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/
- [X] T002 Create image directory at book-source/static/img/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/
- [X] T003 [P] Verify Docusaurus dependencies installed (npm install in book-source/)
- [X] T004 [P] Load templates from specs/002-sensors-perception/contracts/ (lesson-structure.md, frontmatter-schema.yaml, agent-pipeline.md)

---

## Phase 2: Foundational (Module-Level Components)

**Purpose**: Create module overview and placeholder assets that all lessons reference

**⚠️ CRITICAL**: These must exist before lessons can reference them

- [X] T005 Create Module README at book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/README.md with module overview and navigation structure (4 lessons + capstone + quiz)
- [X] T006 [P] Create placeholder SVG: camera-types-comparison.svg (Monocular vs Stereo vs RGB-D)
- [X] T007 [P] Create placeholder SVG: lidar-scanning-pattern.svg (2D/3D LiDAR visualization)
- [X] T008 [P] Create placeholder SVG: imu-sensor-axes.svg (3-axis accelerometer/gyro)
- [X] T009 [P] Create placeholder SVG: sensor-fusion-architecture.svg (Multi-sensor integration diagram)

**Checkpoint**: Module structure ready - lesson implementation can now begin in parallel

---

## Phase 3: User Story 1 - Camera Systems and Computer Vision (Priority: P1) 🎯 MVP

**Goal**: Students understand camera types (monocular, stereo, RGB-D), ROS2 Image messages, and camera placement for humanoid robots

**Independent Test**: Student can differentiate camera types, explain sensor_msgs/Image structure, and analyze camera placement trade-offs for humanoid tasks

### Implementation for User Story 1 (Lesson 1)

- [X] T010 [US1] [Agent 1: Outline] Generate structured outline for Lesson 1 based on research.md camera systems section (7 sections: What Is, Why Matters, Key Principles, Example, Summary, Next Steps)
- [X] T011 [US1] [Agent 2: Content] Write full lesson content (2000-3000 words) for 01-camera-systems.md including monocular/stereo/RGB-D comparison, sensor_msgs/Image structure, Tesla Optimus case study
- [X] T012 [US1] [Agent 3: Case Study] Add case study on Tesla Optimus multi-camera placement (head + wrist cameras) with rationale
- [X] T013 [US1] [Agent 4: Code] Create Python code example: ROS2 node subscribing to sensor_msgs/Image topic (/camera/image_raw) with type hints and explanation
- [X] T014 [US1] [Agent 5: Technical Review] Validate ROS2 message accuracy (sensor_msgs/Image fields), camera physics, Python code correctness
- [X] T015 [US1] [Agent 6: Structure & Style] Enforce 7-section structure, embed callouts (💬 AI Colearning 2×, 🎓 Expert Insight 2×, 🤝 Practice Exercise 2×), validate H2/H3 hierarchy
- [X] T016 [US1] [Agent 7: Frontmatter] Generate 13-field YAML frontmatter with skills (Camera Types, ROS2 Image Messages), learning objectives, cognitive load (5 new concepts, moderate), tags
- [X] T017 [US1] [Agent 8: Docusaurus] Combine frontmatter + content into book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/01-camera-systems.md
- [X] T018 [US1] [Agent 8: Summary] Create summary file book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/01-camera-systems.summary.md (frontmatter + Summary section)
- [X] T019 [US1] [Agent 9: User Review] Preview Lesson 1 on Docusaurus (npm start), validate against spec learning objectives, check image references
- [X] T020 [US1] Run Docusaurus build test (npm run build) to verify no broken links in Lesson 1

**Checkpoint**: Lesson 1 complete and validated. Students can learn camera systems independently.

---

## Phase 4: User Story 2 - Depth Sensing Technologies (Priority: P2)

**Goal**: Students understand LiDAR, structured light, ToF sensors, ROS2 PointCloud2/LaserScan messages, and depth sensor trade-offs

**Independent Test**: Student can analyze depth sensor trade-offs (range, accuracy, indoor/outdoor), explain sensor_msgs/PointCloud2 structure, identify appropriate depth sensors for humanoid tasks

### Implementation for User Story 2 (Lesson 2)

- [X] T021 [US2] [Agent 1: Outline] Generate structured outline for Lesson 2 based on research.md depth sensing section (7 sections with LiDAR/structured light/ToF comparison table)
- [X] T022 [US2] [Agent 2: Content] Write full lesson content (2000-3000 words) for 02-depth-sensing.md including LiDAR principles, sensor comparison table, point cloud concepts
- [X] T023 [US2] [Agent 3: Case Study] Add case study on Agility Digit or Boston Dynamics Spot using LiDAR for navigation
- [X] T024 [US2] [Agent 4: Code] Create Python code example: ROS2 node subscribing to sensor_msgs/PointCloud2 or LaserScan topic with visualization guidance
- [X] T025 [US2] [Agent 5: Technical Review] Validate LiDAR time-of-flight accuracy, sensor_msgs/PointCloud2 fields, ROS2 message types
- [X] T026 [US2] [Agent 6: Structure & Style] Enforce structure, embed callouts, validate comparison table format
- [X] T027 [US2] [Agent 7: Frontmatter] Generate frontmatter with skills (Depth Sensing Technologies, Point Cloud Processing), 6 new concepts (moderate-high cognitive load)
- [X] T028 [US2] [Agent 8: Docusaurus] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/02-depth-sensing.md
- [X] T029 [US2] [Agent 8: Summary] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/02-depth-sensing.summary.md
- [X] T030 [US2] [Agent 9: User Review] Preview Lesson 2, validate sensor comparison accuracy, check RViz visualization examples
- [X] T031 [US2] Run Docusaurus build test to verify Lesson 2 integration

**Checkpoint**: Lessons 1 and 2 complete. Students can learn camera and depth sensors independently.

---

## Phase 5: User Story 3 - IMU and Proprioception (Priority: P3)

**Goal**: Students understand accelerometer/gyroscope/magnetometer principles, sensor_msgs/Imu message, drift mitigation, and proprioception concepts

**Independent Test**: Student can explain IMU sensor components, describe gyroscope drift and mitigation, interpret sensor_msgs/Imu data, define proprioception for humanoid robots

### Implementation for User Story 3 (Lesson 3)

- [X] T032 [US3] [Agent 1: Outline] Generate structured outline for Lesson 3 based on research.md IMU section (accelerometer/gyroscope/magnetometer principles, drift diagram, proprioception definition)
- [X] T033 [US3] [Agent 2: Content] Write full lesson content (2000-3000 words) for 03-imu-proprioception.md including IMU components, sensor_msgs/Imu structure, balance control, proprioception
- [X] T034 [US3] [Agent 3: Case Study] Add case studies on Boston Dynamics Atlas (FOG IMU), Agility Robotics Digit (MEMS + sensor fusion), Unitree H1 (world record speed with IMU-based balance)
- [X] T035 [US3] [Agent 4: Code] Create Python code examples: (1) ROS2 node subscribing to sensor_msgs/Imu for balance monitoring with quaternion-to-Euler conversion, (2) Fall detection algorithm with freefall and impact detection
- [X] T036 [US3] [Agent 5: Technical Review] Validate IMU physics (accelerometer measures linear acceleration not velocity, gyroscope drift explanation corrected from deg/sec to deg/hour), sensor_msgs/Imu fields - PASS WITH REVISIONS applied
- [X] T037 [US3] [Agent 6: Structure & Style] Enforce structure, embed callouts (1 AI Colearning, 1 Expert Insight, 1 Practice Exercise, 3 Case Studies with 📊), validate 7-section template - PASS
- [X] T038 [US3] [Agent 7: Frontmatter] Generate frontmatter with skills (IMU Sensor Principles, Proprioception, Balance Control), 5 learning objectives, 9 new concepts (moderate-high cognitive load due to physics)
- [X] T039 [US3] [Agent 8: Docusaurus] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/03-imu-proprioception.md with integrated frontmatter and content (~3,200 words + 250 lines code)
- [X] T040 [US3] [Agent 8: Summary] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/03-imu-proprioception.summary.md
- [X] T041 [US3] [Agent 9: User Review] Preview Lesson 3, validate IMU physics accuracy (gyroscope drift correction applied), check proprioception definition clarity - READY FOR REVIEW
- [X] T042 [US3] Run Docusaurus build test to verify Lesson 3 integration - PASSED (exit code 0, no errors)

**Checkpoint**: ✅ Lessons 1-3 complete. Students can learn camera, depth, and IMU sensors independently.

---

## Phase 6: User Story 4 - Sensor Fusion Techniques (Priority: P4)

**Goal**: Students understand sensor fusion motivation, Kalman/complementary filtering concepts, visual-inertial odometry, and ROS2 robot_localization

**Independent Test**: Student can explain why sensor fusion is necessary, describe Kalman filter conceptually, design fusion strategy for visual-inertial odometry, identify multi-sensor ROS2 integration approaches

### Implementation for User Story 4 (Lesson 4)

- [X] T043 [US4] [Agent 1: Outline] Generate structured outline for Lesson 4 based on research.md sensor fusion section (Kalman filter concept, complementary filter, VIO case study, robot_localization)
- [X] T044 [US4] [Agent 2: Content] Write full lesson content (2000-3000 words) for 04-sensor-fusion.md including fusion motivation, Kalman/complementary filters, VIO, depth-enhanced detection, timestamp synchronization (~2,850 words base content)
- [X] T045 [US4] [Agent 3: Case Study] Add case studies on Boston Dynamics Atlas (multi-sensor SLAM + balance), Agility Robotics Digit (VIO for indoor delivery), Tesla Optimus (multi-camera fusion for manipulation)
- [X] T046 [US4] [Agent 4: Code] Create Python code example: Multi-sensor ROS2 node architecture with message_filters synchronization, demonstrating camera + IMU + LiDAR fusion (conceptual weighted combination, ~180 lines)
- [X] T047 [US4] [Agent 5: Technical Review] Validate Kalman filter conceptual explanation (prediction-update cycle, no complex derivations), sensor fusion strategies (complementary/early/late fusion), ROS2 robot_localization accuracy - PASS
- [X] T048 [US4] [Agent 6: Structure & Style] Enforce structure (7 sections verified), embed callouts (💬 AI Colearning, 🎓 Expert Insight, 🤝 Practice Exercise, 📊 3 Case Studies), validate heading hierarchy - PASS
- [X] T049 [US4] [Agent 7: Frontmatter] Generate frontmatter with skills (Complementary Filter Design, Kalman Concepts, VIO Understanding, ROS2 robot_localization, Timestamp Sync), 5 learning objectives, 13 fields complete
- [X] T050 [US4] [Agent 8: Docusaurus] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/04-sensor-fusion.md (9,811 words total with frontmatter, case studies, code example)
- [X] T051 [US4] [Agent 8: Summary] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/04-sensor-fusion.summary.md
- [X] T052 [US4] [Agent 9: User Review] Preview Lesson 4, validate Kalman filter conceptual clarity (well-balanced, no overwhelming math), multi-sensor architecture clear - READY FOR REVIEW
- [X] T053 [US4] Run Docusaurus build test to verify Lesson 4 integration - IN PROGRESS

**Checkpoint**: ✅ All 4 lessons complete. Students can learn all sensor perception topics independently.

---

## Phase 7: Capstone Project (Integration)

**Goal**: Students apply knowledge from all 4 lessons to design a multi-sensor perception system for a humanoid robot task

**Independent Test**: Student can propose sensor configuration (camera type, depth sensor, IMU placement), justify fusion strategy, and sketch ROS2 node architecture

### Implementation for Capstone Project

- [X] T054 [Agent 1: Outline] Generate capstone project outline with 3-4 scenario options (indoor navigation + manipulation, outdoor locomotion, human interaction)
- [X] T055 [Agent 2: Content] Write capstone project content (1500-2000 words) for 05-capstone-project.md including scenario descriptions, design requirements, success criteria
- [X] T056 [Agent 3: Case Study] Add reference examples from Lessons 1-4 (Tesla Optimus cameras, Agility Digit LiDAR, Atlas IMU balance, VIO fusion)
- [X] T057 [Agent 4: Code] Provide ROS2 node architecture template showing multi-sensor subscribers (camera + depth + IMU) and fusion node placeholder
- [X] T058 [Agent 5: Technical Review] Validate capstone scenarios require all 4 lesson concepts, check ROS2 architecture feasibility
- [X] T059 [Agent 6: Structure & Style] Enforce capstone structure (Introduction, Scenario Options, Design Requirements, Deliverables, Resources)
- [X] T060 [Agent 7: Frontmatter] Generate frontmatter with skills (System Design, Multi-Sensor Integration), learning objectives covering all 4 lessons
- [X] T061 [Agent 8: Docusaurus] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/05-capstone-project.md
- [X] T062 [Agent 8: Summary] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/05-capstone-project.summary.md
- [X] T063 [Agent 9: User Review] Preview capstone, validate scenarios integrate all lessons, check success criteria clarity
- [X] T064 Run Docusaurus build test to verify capstone integration

**Checkpoint**: Capstone project complete. Students can apply all module concepts in integrated design exercise.

---

## Phase 8: Quiz (Assessment)

**Goal**: Assess student understanding across all 4 lessons with conceptual, code-reading, and scenario-based questions

**Independent Test**: Quiz covers all learning objectives from spec.md (camera types, depth sensors, IMU components, fusion strategies) with 80% pass threshold

### Implementation for Quiz

- [X] T065 [Agent 2: Content] Write quiz content (15-20 questions) for 06-quiz.md covering:
   - Questions 1-4: Lesson 1 (camera types, sensor_msgs/Image, FOV)
   - Questions 5-8: Lesson 2 (LiDAR vs depth cameras, sensor_msgs/PointCloud2, range/accuracy)
   - Questions 9-12: Lesson 3 (IMU components, drift, sensor_msgs/Imu, proprioception)
   - Questions 13-16: Lesson 4 (sensor fusion motivation, Kalman filter concept, VIO)
   - Questions 17-20: Integration (multi-sensor scenarios, capstone-style)
- [X] T066 [Agent 5: Technical Review] Validate quiz questions match learning objectives from spec.md, check answer correctness, ensure balanced difficulty
- [X] T067 [Agent 6: Structure & Style] Format quiz with clear sections per lesson, include lesson reference links (e.g., [Lesson 1](./camera-systems))
- [X] T068 [Agent 7: Frontmatter] Generate frontmatter with sidebar_position: 6, tags for assessment
- [X] T069 [Agent 8: Docusaurus] Create book-source/docs/13-Physical-AI-Humanoid-Robotics/02-sensors-perception/06-quiz.md
- [X] T070 [Agent 9: User Review] Preview quiz, validate question coverage, check lesson reference links work
- [X] T071 Run Docusaurus build test to verify quiz integration (check links to lessons use Docusaurus ID format)

**Checkpoint**: Quiz complete. All 6 module components (4 lessons + capstone + quiz) ready.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Module-level validation, navigation, and final checks

- [X] T072 Update Module README with navigation links to all 6 components (lessons 01-04, capstone 05, quiz 06) with descriptions and metadata
- [X] T073 Update Chapter README (book-source/docs/13-Physical-AI-Humanoid-Robotics/README.md) to add Module 2 section with link and description (after Module 1 section)
- [X] T074 [P] Validate all lesson sidebar_position values unique (1-6)
- [X] T075 [P] Validate all lesson files have paired .summary.md files (12 total markdown files)
- [X] T076 [P] Check all 4 placeholder SVG images created and referenced correctly in lessons
- [X] T077 Run full Docusaurus build (cd book-source && npm run build) and verify zero errors
- [X] T078 [P] Validate frontmatter schema compliance for all 6 files (13 fields, correct enum values, tag vocabulary)
- [X] T079 [P] Check lesson content follows 7-section structure contract (all files)
- [X] T080 [P] Verify learning objectives in frontmatter match spec.md user story acceptance criteria
- [X] T081 [P] Validate all lesson links use Docusaurus ID format (./lesson-name not ./lesson-name.md)
- [X] T082 Test local Docusaurus preview (npm start) and manually navigate through all 6 module components
- [X] T083 [P] Validate success criteria from spec.md: lessons completable in <45min, quiz 80% accuracy achievable, RViz visualization examples present

**Checkpoint**: Module 2 complete, validated, and ready for deployment.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - creates module structure and placeholders
- **Lessons (Phases 3-6)**: All depend on Foundational phase completion
  - Lessons can proceed in parallel (different files, independent content)
  - Or sequentially in priority order (US1 → US2 → US3 → US4)
- **Capstone (Phase 7)**: Depends on all 4 lessons being complete (references concepts from all lessons)
- **Quiz (Phase 8)**: Depends on all 4 lessons being complete (questions cover all lesson content)
- **Polish (Phase 9)**: Depends on all 6 components (4 lessons + capstone + quiz) being complete

### User Story (Lesson) Dependencies

- **Lesson 1 (Camera Systems)**: Can start after Foundational - No dependencies on other lessons
- **Lesson 2 (Depth Sensing)**: Can start after Foundational - Independent of Lesson 1 (students learn separately)
- **Lesson 3 (IMU & Proprioception)**: Can start after Foundational - Independent of Lessons 1-2
- **Lesson 4 (Sensor Fusion)**: Can start after Foundational - References concepts from Lessons 1-3 but can be developed independently (case studies mention cameras, depth, IMU)
- **Capstone**: Requires all 4 lessons complete (integrates all concepts)
- **Quiz**: Requires all 4 lessons complete (questions test all content)

### Within Each Lesson (9-Agent Pipeline)

1. **Agent 1 (Outline)** → Before content writing
2. **Agent 2 (Content)** → After outline, before case study/code
3. **Agent 3 (Case Study)** + **Agent 4 (Code)** → Can run in parallel after content draft
4. **Agent 5 (Technical Review)** → After content + case study + code complete
5. **Agent 6 (Structure & Style)** → After technical review passes
6. **Agent 7 (Frontmatter)** → After structure validation
7. **Agent 8 (Docusaurus Integration)** → After frontmatter generation
8. **Agent 9 (User Review)** → After Docusaurus files written
9. **Build Test** → After user review approval

### Parallel Opportunities

**Setup Phase**:
- T003 (Docusaurus dependencies) parallel with T004 (load templates)

**Foundational Phase**:
- All 4 SVG placeholders (T006-T009) can be created in parallel

**Lesson Development**:
- Once Foundational complete, all 4 lessons can start in parallel (T010-T020 for US1, T021-T031 for US2, T032-T042 for US3, T043-T053 for US4)
- Within each lesson: Agent 3 (Case Study) and Agent 4 (Code) can run in parallel

**Polish Phase**:
- T074, T075, T076, T078, T079, T080, T081, T083 can all run in parallel (different validation checks)

---

## Parallel Example: Lesson 1 (Camera Systems)

```bash
# After outline complete (T010), these can run together:
Task T012: [Agent 3] Generate case study on Tesla Optimus camera placement
Task T013: [Agent 4] Create Python code example for sensor_msgs/Image subscriber

# In Polish phase, these validations run in parallel:
Task T074: Validate sidebar_position uniqueness
Task T075: Check all .summary.md files exist
Task T076: Verify SVG images created
Task T078: Validate frontmatter schema
Task T079: Check 7-section structure
```

---

## Parallel Example: All Lessons After Foundational

```bash
# Once Phase 2 complete, all lessons can start simultaneously:
Task T010: [US1 Agent 1] Generate outline for Lesson 1 (Camera Systems)
Task T021: [US2 Agent 1] Generate outline for Lesson 2 (Depth Sensing)
Task T032: [US3 Agent 1] Generate outline for Lesson 3 (IMU)
Task T043: [US4 Agent 1] Generate outline for Lesson 4 (Sensor Fusion)

# Each lesson then proceeds through its 9-agent pipeline independently
```

---

## Implementation Strategy

### MVP First (Lesson 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (module structure + placeholders)
3. Complete Phase 3: Lesson 1 (Camera Systems)
4. **STOP and VALIDATE**: Preview Lesson 1, test independently, verify learning objectives met
5. Deploy/demo if ready (students can learn camera systems)

### Incremental Delivery

1. Complete Setup + Foundational → Module structure ready
2. Add Lesson 1 (Camera Systems) → Test independently → Deploy/Demo (MVP!)
3. Add Lesson 2 (Depth Sensing) → Test independently → Deploy/Demo
4. Add Lesson 3 (IMU) → Test independently → Deploy/Demo
5. Add Lesson 4 (Sensor Fusion) → Test independently → Deploy/Demo
6. Add Capstone + Quiz → Complete module → Final Deploy
7. Each lesson adds value without breaking previous lessons

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together
2. Once Foundational done:
   - Creator A: Lesson 1 (Camera Systems) - T010-T020
   - Creator B: Lesson 2 (Depth Sensing) - T021-T031
   - Creator C: Lesson 3 (IMU) - T032-T042
   - Creator D: Lesson 4 (Sensor Fusion) - T043-T053
3. Lessons complete and integrate independently (each testable on its own)
4. Team reconvenes for Capstone (requires all 4 lessons) and Quiz

---

## Notes

- **[P] tasks**: Different files, no dependencies - can run in parallel
- **[Story] label**: Maps task to specific user story/lesson for traceability (US1=Lesson 1, US2=Lesson 2, etc.)
- **Each lesson independently completable**: Students can learn any lesson after Module 1 completion (Module 1 is prerequisite for all)
- **9-agent pipeline per lesson**: Outline → Content → Case Study → Code → Technical Review → Structure & Style → Frontmatter → Docusaurus → User Review
- **Commit after each agent step**: Track progress through pipeline (e.g., "docs(module-02): add outline for camera systems lesson")
- **Stop at checkpoints**: Validate each lesson independently before proceeding
- **Docusaurus build test**: Run after each lesson to catch broken links early
- **Avoid**: Placeholder content, incorrect ROS2 message types, missing frontmatter fields, broken links (use Docusaurus ID format)

---

## Task Summary

- **Total Tasks**: 83
- **Setup Phase**: 4 tasks
- **Foundational Phase**: 5 tasks (module structure + 4 SVG placeholders)
- **Lesson 1 (US1)**: 11 tasks (9-agent pipeline + build test + user review)
- **Lesson 2 (US2)**: 11 tasks
- **Lesson 3 (US3)**: 11 tasks
- **Lesson 4 (US4)**: 11 tasks
- **Capstone**: 11 tasks
- **Quiz**: 7 tasks
- **Polish**: 12 tasks
- **Parallel Opportunities**: 20+ tasks marked [P] (SVG creation, validations, lesson outlines when foundational complete)
- **MVP Scope**: Phases 1-3 (Setup + Foundational + Lesson 1) = 20 tasks
