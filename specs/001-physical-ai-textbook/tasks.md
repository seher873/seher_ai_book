# Tasks: Physical AI Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `content/` at repository root
- **Configuration**: `docusaurus.config.js` and related config files
- **Assets**: `static/` for images, diagrams, and other static resources
- **Code examples**: `content/examples/` for textbook code examples

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Set up Docusaurus project structure per implementation plan
- [ ] T002 Initialize Node.js project with Docusaurus v3.x dependencies
- [ ] T003 [P] Configure linting and formatting tools for Markdown content

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T004 Set up Docusaurus configuration for textbook structure
- [ ] T005 [P] Create content directory structure per textbook chapters
- [ ] T006 [P] Configure navigation sidebar with textbook sections
- [ ] T007 Create base Markdown templates for textbook content
- [ ] T008 Configure diagram rendering for textbook content
- [ ] T009 Setup content versioning and cross-referencing system

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Learning Physical AI Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Create foundational textbook content covering ROS2 basics, perception systems, and simulation environments so students can learn physical AI fundamentals.

**Independent Test**: Students can read Chapter 1 on ROS2 basics and gain foundational knowledge sufficient to understand the ROS2 architecture and basic commands.

### Implementation for User Story 1

- [ ] T010 [P] [US1] Create Chapter 1: Introduction to Physical AI and ROS2 Framework content in content/chapter-1/index.md
- [ ] T011 [P] [US1] Create Chapter 2: Robot Perception Systems content in content/chapter-2/index.md
- [ ] T012 [P] [US1] Create Chapter 3: Simulation Environments - Gazebo and Unity content in content/chapter-3/index.md
- [ ] T013 [P] [US1] Create Module 1: ROS2 Deep Dive content in content/module-1/index.md
- [ ] T014 [P] [US1] Create Module 2: Advanced Perception Techniques content in content/module-2/index.md
- [ ] T015 [P] [US1] Create Week 1 curriculum content in content/weekly-plan/week-1.md
- [ ] T016 [P] [US1] Create Week 2 curriculum content in content/weekly-plan/week-2.md
- [ ] T017 [P] [US1] Create Week 3 curriculum content in content/weekly-plan/week-3.md
- [ ] T018 [P] [US1] Create Week 4 curriculum content in content/weekly-plan/week-4.md
- [ ] T019 [P] [US1] Create Lab 1.1: ROS2 Installation and Basic Commands in content/labs/lab-1.1.md
- [ ] T020 [P] [US1] Create Lab 1.2: Creating Your First Publisher and Subscriber Nodes in content/labs/lab-1.2.md
- [ ] T021 [P] [US1] Create Lab 3.1: Processing Camera Images in ROS2 in content/labs/lab-3.1.md
- [ ] T022 [P] [US1] Create Lab 4.1: Creating a Simple Differential Drive Robot in content/labs/lab-4.1.md
- [ ] T023 [US1] Add diagram placeholders for Chapter 1 (Physical AI Architecture Overview) to content/chapter-1/diagrams/
- [ ] T024 [US1] Add diagram placeholders for Chapter 2 (Perception Pipeline Architecture) to content/chapter-2/diagrams/
- [ ] T025 [US1] Add diagram placeholders for Chapter 3 (Gazebo Architecture) to content/chapter-3/diagrams/
- [ ] T026 [US1] Add code examples for ROS2 fundamentals in content/examples/ros2-basics/
- [ ] T027 [US1] Add code examples for perception in content/examples/perception/
- [ ] T028 [US1] Create glossary terms for fundamental concepts in content/appendices/glossary.md
- [ ] T029 [US1] Add self-assessment questions for Chapter 1 in content/chapter-1/exercises.md
- [ ] T030 [US1] Add self-assessment questions for Chapter 2 in content/chapter-2/exercises.md
- [ ] T031 [US1] Add self-assessment questions for Chapter 3 in content/chapter-3/exercises.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Instructor Planning Course Curriculum (Priority: P2)

**Goal**: Complete the 14-week curriculum plan and associated materials so instructors can structure their semester-long course.

**Independent Test**: Instructors can follow the week-by-week plan and assign corresponding lab exercises to teach physical AI concepts effectively.

### Implementation for User Story 2

- [ ] T032 [P] [US2] Create Week 5 curriculum content in content/weekly-plan/week-5.md
- [ ] T033 [P] [US2] Create Week 6 curriculum content in content/weekly-plan/week-6.md
- [ ] T034 [P] [US2] Create Week 7 curriculum content in content/weekly-plan/week-7.md
- [ ] T035 [P] [US2] Create Week 8 curriculum content in content/weekly-plan/week-8.md
- [ ] T036 [P] [US2] Create Week 9 curriculum content in content/weekly-plan/week-9.md
- [ ] T037 [P] [US2] Create Week 10 curriculum content in content/weekly-plan/week-10.md
- [ ] T038 [P] [US2] Create Week 11 curriculum content in content/weekly-plan/week-11.md
- [ ] T039 [P] [US2] Create Week 12 curriculum content in content/weekly-plan/week-12.md
- [ ] T040 [P] [US2] Create Week 13 curriculum content in content/weekly-plan/week-13.md
- [ ] T041 [P] [US2] Create Week 14 curriculum content in content/weekly-plan/week-14.md
- [ ] T042 [P] [US2] Create Lab 5.1: Unity Robot Navigation with ML-Agents in content/labs/lab-5.1.md
- [ ] T043 [P] [US2] Create Lab 6.1: Running Isaac Sample Applications in content/labs/lab-6.1.md
- [ ] T044 [P] [US2] Create Lab 9.1: Implementing PID Position Controller in content/labs/lab-9.1.md
- [ ] T045 [P] [US2] Create Lab 10.1: Implementing RRT Planner for Mobile Robot in content/labs/lab-10.1.md
- [ ] T046 [P] [US2] Create Lab 11.1: Building 2D Map with TurtleBot in content/labs/lab-11.1.md
- [ ] T047 [P] [US2] Create Lab 12.1: Inverse Kinematics with MoveIt! in content/labs/lab-12.1.md
- [ ] T048 [P] [US2] Create Lab 13.1: Integrated Navigation and Manipulation Task in content/labs/lab-13.1.md
- [ ] T049 [US2] Create instructor's guide for course structure in content/instructor-guide/course-structure.md
- [ ] T050 [US2] Create instructor's guide for lab assessment in content/instructor-guide/lab-assessment.md
- [ ] T051 [US2] Develop course syllabus template in content/instructor-guide/syllabus-template.md
- [ ] T052 [US2] Create assessment rubrics for coursework in content/assessments/rubrics.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Practitioner Implementing Physical AI Solutions (Priority: P3)

**Goal**: Complete advanced content modules covering Isaac platform, VLA models, advanced control systems, and integration patterns so practitioners can implement real-world physical AI solutions.

**Independent Test**: Engineers can reference specific modules on Isaac or VLA implementations and successfully apply them to their projects.

### Implementation for User Story 3

- [ ] T053 [P] [US3] Create Chapter 4: Isaac Robotics Platform content in content/chapter-4/index.md
- [ ] T054 [P] [US3] Create Chapter 5: Vision Language Action (VLA) Models content in content/chapter-5/index.md
- [ ] T055 [P] [US3] Create Chapter 6: Robot Control and Manipulation content in content/chapter-6/index.md
- [ ] T056 [P] [US3] Create Chapter 7: Navigation Systems content in content/chapter-7/index.md
- [ ] T057 [P] [US3] Create Chapter 8: Integration and System Design content in content/chapter-8/index.md
- [ ] T058 [P] [US3] Create Chapter 9: Ethics and Social Implications content in content/chapter-9/index.md
- [ ] T059 [P] [US3] Create Module 3: Simulation and Training Pipelines content in content/module-3/index.md
- [ ] T060 [P] [US3] Create Module 4: Isaac Platform Mastery content in content/module-4/index.md
- [ ] T061 [P] [US3] Create Module 5: Vision-Language-Action Implementation content in content/module-5/index.md
- [ ] T062 [P] [US3] Create Module 6: Advanced Control Systems content in content/module-6/index.md
- [ ] T063 [P] [US3] Create Lab 7.1: Building a 3D Object Detection Pipeline in content/labs/lab-7.1.md
- [ ] T064 [P] [US3] Create Lab 8.1: Running Pretrained VLA Models with Simulated Robots in content/labs/lab-8.1.md
- [ ] T065 [P] [US3] Create Lab 14.1: Performance Evaluation of Developed System in content/labs/lab-14.1.md
- [ ] T066 [US3] Add diagram placeholders for Chapter 4 (Isaac Architecture Overview) to content/chapter-4/diagrams/
- [ ] T067 [US3] Add diagram placeholders for Chapter 5 (VLA Model Architecture) to content/chapter-5/diagrams/
- [ ] T068 [US3] Add diagram placeholders for Chapter 8 (Full System Architecture) to content/chapter-8/diagrams/
- [ ] T069 [US3] Add code examples for Isaac platform in content/examples/isaac/
- [ ] T070 [US3] Add code examples for VLA models in content/examples/vla/
- [ ] T071 [US3] Add code examples for integration patterns in content/examples/integration/
- [ ] T072 [US3] Create case studies for real-world implementations in content/case-studies/
- [ ] T073 [US3] Create troubleshooting guide in content/appendices/troubleshooting.md
- [ ] T074 [US3] Create additional resources appendix in content/appendices/additional-resources.md
- [ ] T075 [US3] Create mathematical foundations appendix in content/appendices/math-foundations.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Hardware/Labs Section

**Goal**: Complete the hardware recommendations and lab equipment sections to support practical implementations.

### Implementation

- [ ] T076 [P] Create Recommended Hardware Platforms section in content/hardware-platforms/index.md  
- [ ] T077 [P] Create Lab Equipment Recommendations section in content/lab-equipment/index.md
- [ ] T078 Create Software Requirements section in content/software-requirements/index.md
- [ ] T079 Create Lab Safety Considerations section in content/lab-safety/index.md
- [ ] T080 [P] Create Lab 2.2: Visualizing LiDAR Point Clouds in content/labs/lab-2.2.md
- [ ] T081 [P] Create Lab 5.2: Comparing Gazebo vs Unity Simulation Results in content/labs/lab-5.2.md
- [ ] T082 [P] Create Lab 6.2: Exploring Isaac Sim Capabilities in content/labs/lab-6.2.md
- [ ] T083 [P] Create Lab 7.2: Integrating Isaac Perception with ROS2 in content/labs/lab-7.2.md
- [ ] T084 [P] Create Lab 8.2: Customizing VLA Inputs for Specific Tasks in content/labs/lab-8.2.md
- [ ] T085 [P] Create Lab 9.2: Cartesian Space Trajectory Following in content/labs/lab-9.2.md
- [ ] T086 [P] Create Lab 10.2: Path Following with Obstacle Avoidance in content/labs/lab-10.2.md
- [ ] T087 [P] Create Lab 11.2: Autonomous Navigation in Known Map in content/labs/lab-11.2.md
- [ ] T088 [P] Create Lab 12.2: Pick and Place with Robot Arm in content/labs/lab-12.2.md
- [ ] T089 [P] Create Lab 13.2: Debugging an Integrated Robotic System in content/labs/lab-13.2.md
- [ ] T090 [P] Create Lab 14.2: System Deployment Planning and Ethics Review in content/labs/lab-14.2.md

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T091 [P] Add glossary terms for advanced concepts in content/appendices/glossary.md
- [ ] T092 [P] Add cross-references between chapters for better navigation
- [ ] T093 [P] Add accessibility features to diagrams and content
- [ ] T094 [P] Add code syntax highlighting and formatting to all examples
- [ ] T095 [P] Add ethical considerations to each technical chapter
- [ ] T096 [P] Add mathematical foundations to relevant chapters
- [ ] T097 [P] Add troubleshooting sections to each module
- [ ] T098 [P] Add assessment questions for each week
- [ ] T099 [P] Add additional resources to each chapter
- [ ] T100 [P] Add links to online communities and forums
- [ ] T101 [P] Add quickstart guide for different technical backgrounds
- [ ] T102 Documentation updates in README.md
- [ ] T103 Run quickstart.md validation to ensure content matches setup
- [ ] T104 Test textbook content rendering in Docusaurus environment
- [ ] T105 Review all content for accuracy by domain experts

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Hardware/Labs Section**: Depends on foundational content
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May reference US1 content but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May reference US1/US2 content but should be independently testable

### Within Each User Story

- Core chapters before modules
- Curriculum plans before detailed labs
- Content before code examples
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All chapters within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members
- All labs can be developed in parallel once prerequisites are established

---

## Parallel Example: User Story 1

```bash
# Launch all chapters for User Story 1 together:
Task: "Create Chapter 1: Introduction to Physical AI and ROS2 Framework content in content/chapter-1/index.md"
Task: "Create Chapter 2: Robot Perception Systems content in content/chapter-2/index.md"
Task: "Create Chapter 3: Simulation Environments - Gazebo and Unity content in content/chapter-3/index.md"

# Launch all modules for User Story 1 together:
Task: "Create Module 1: ROS2 Deep Dive content in content/module-1/index.md"
Task: "Create Module 2: Advanced Perception Techniques content in content/module-2/index.md"

# Launch all labs for User Story 1 together:
Task: "Create Lab 1.1: ROS2 Installation and Basic Commands in content/labs/lab-1.1.md"
Task: "Create Lab 1.2: Creating Your First Publisher and Subscriber Nodes in content/labs/lab-1.2.md"
Task: "Create Lab 3.1: Processing Camera Images in ROS2 in content/labs/lab-3.1.md"
Task: "Create Lab 4.1: Creating a Simple Differential Drive Robot in content/labs/lab-4.1.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Hardware/Labs → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: Hardware/Labs
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence