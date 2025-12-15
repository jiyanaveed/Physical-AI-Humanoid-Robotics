# Tasks: Module 1 - ROS 2 and Robotics Fundamentals

**Input**: `.specify/memory/MODULE_1_PLAN.md`

---

## Phase 1: Research

- [ ] T001 [P] [M1] Research and document best analogies for explaining ROS 2 concepts in `research/ros2_concepts_research.md`.
- [ ] T002 [P] [M1] Research and document best practices for teaching URDF in `research/urdf_teaching_research.md`.

---

## Phase 2: Foundation

- [ ] T003 [M1] Create the directory `book/docs/module-1-ros2-fundamentals/`.
- [ ] T004 [M1] Create placeholder file `book/docs/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services.md` with Docusaurus front matter.
- [ ] T005 [M1] Create placeholder file `book/docs/module-1-ros2-fundamentals/chapter-1-2-python-agents-ros.md` with Docusaurus front matter.
- [ ] T006 [M1] Create placeholder file `book/docs/module-1-ros2-fundamentals/chapter-1-3-urdf-humanoid-kinematics.md` with Docusaurus front matter.
- [ ] T007 [M1] Set up Python package `src/module1_examples` for code examples with `pytest` config.

---

## Phase 3: Analysis (Content Creation)

### Chapter 1.1: Nodes, Topics, & Services
- [ ] T008 [US1] [M1] Write the full text explaining nodes, topics, and services in `book/docs/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services.md`.
- [ ] T009 [P] [US1] [M1] Create commented code example for a publisher node in `src/module1_examples/publisher.py`.
- [ ] T010 [P] [US1] [M1] Create commented code example for a subscriber node in `src/module1_examples/subscriber.py`.
- [ ] T011 [P] [US1] [M1] Create commented code example for a service server/client in `src/module1_examples/service.py`.
- [ ] T012 [US1] [M1] Develop hands-on exercises for Chapter 1.1 and add them to the markdown file.
- [ ] T013 [US1] [M1] Develop the capstone project for Chapter 1.1 and add it to the markdown file.

### Chapter 1.2: Python Agents & ROS
- [ ] T014 [US2] [M1] Write the content for bridging Python agents with ROS controllers in `book/docs/module-1-ros2-fundamentals/chapter-1-2-python-agents-ros.md`.
- [ ] T015 [P] [US2] [M1] Create code example for an agent subscribing to sensor topics in `src/module1_examples/agent_subscriber.py`.
- [ ] T016 [P] [US2] [M1] Create code example for an agent publishing control commands in `src/module1_examples/agent_publisher.py`.
- [ ] T017 [US2] [M1] Create code example for an action server/client in `src/module1_examples/action_example.py`.

### Chapter 1.3: Advanced URDF
- [ ] T018 [US3] [M1] Write the guide to advanced URDF and xacro in `book/docs/module-1-ros2-fundamentals/chapter-1-3-urdf-humanoid-kinematics.md`.
- [ ] T019 [P] [US3] [M1] Create an example URDF for a simple arm in `src/module1_examples/urdf/simple_arm.urdf`.
- [ ] T020 [P] [US3] [M1] Create an example xacro for a torso with arms in `src/module1_examples/urdf/torso_with_arms.xacro`.
- [ ] T021 [US3] [M1] Create an example xacro for a full humanoid upper body in `src/module1_examples/urdf/humanoid_upper_body.xacro`.

---

## Phase 4: Synthesis (Review & Polish)

- [ ] T022 [M1] Perform peer review of all content in `book/docs/module-1-ros2-fundamentals/`.
- [ ] T023 [P] [M1] Write `pytest` tests for all Python code examples in `src/module1_examples/tests/`.
- [ ] T024 [P] [M1] Write `launch_testing` tests for all launch files.
- [ ] T025 [P] [M1] Validate all URDF/xacro files using `check_urdf`.
- [ ] T026 [M1] Perform a final read-through of the entire module.
- [ ] T027 [M1] Finalize all diagrams and ensure they are correctly embedded in the markdown files.
- [ ] T028 [M1] **Constitution Check**: Verify all content and code in Module 1 against `.specify/memory/constitution.md`.
- [ ] T029 [M1] **Docusaurus Compliance**: Ensure all markdown files render correctly in Docusaurus.
