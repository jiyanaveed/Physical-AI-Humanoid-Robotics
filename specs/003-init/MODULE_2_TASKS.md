# Tasks: Module 2: Core AI Algorithms for Robotics

**Input**: Design documents from `/specs/[###-feature-name]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Research

**Purpose**: Initial investigation and resource compilation.

- [ ] T001 [M2] Research intuitive visualization methods for graph search algorithms (A*, Dijkstra's) and document findings in `.specify/memory/MODULE_2_RESEARCH.md`
- [ ] T002 [M2] Compile a list of academic and blog resources for EKF SLAM, GraphSLAM, DQN, and PPO theory, document findings in `.specify/memory/MODULE_2_RESEARCH.md`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T003 [M2] Create module directory `book/docs/module-2-core-ai-algorithms/`
- [ ] T004 [M2] Create placeholder Markdown for Chapter 2.1 in `book/docs/module-2-core-ai-algorithms/chapter-2-1-path-planning.md`
- [ ] T005 [M2] Create placeholder Markdown for Chapter 2.2 in `book/docs/module-2-core-ai-algorithms/chapter-2-2-slam-principles.md`
- [ ] T006 [M2] Create placeholder Markdown for Chapter 2.3 in `book/docs/module-2-core-ai-algorithms/chapter-2-3-reinforcement-learning.md`
- [ ] T007 [M2] Create dedicated Python package `src/module2_examples/`
- [ ] T008 [M2] Create `__init__.py` for `src/module2_examples/`
- [ ] T009 [M2] Create `pytest.ini` for `src/module2_examples/`
- [ ] T010 [M2] Add required dependencies (pytest, pytorch, numpy, scipy) to `src/module2_examples/requirements.txt` and `pyproject.toml`
- [ ] T011 [M2] Create `src/module2_examples/tests/` directory for code examples

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Chapter 2.1: Path Planning

**Goal**: Deliver fully explained and implemented path planning algorithms (A*, Dijkstra's, RRT) with a ROS 2 wrapper.

**Independent Test**: Python unit tests pass for all path planning algorithms; ROS 2 wrapper can be demonstrated with a simple environment.

### Implementation for User Story 1

- [ ] T012 [M2] [US1] Write full text explanation for path planning algorithms in `book/docs/module-2-core-ai-algorithms/chapter-2-1-path-planning.md`
- [ ] T013 [M2] [US1] Implement A* algorithm in `src/module2_examples/path_planning/a_star.py`
- [ ] T014 [M2] [US1] Implement Dijkstra's algorithm in `src/module2_examples/path_planning/dijkstra.py`
- [ ] T015 [M2] [US1] Implement RRT algorithm in `src/module2_examples/path_planning/rrt.py`
- [ ] T016 [M2] [US1] Develop ROS 2 wrapper for path planning algorithms in `src/module2_examples/path_planning/ros2_wrapper.py`
- [ ] T017 [M2] [US1] Create unit tests for A*, Dijkstra's, and RRT in `src/module2_examples/tests/test_path_planning.py`

**Checkpoint**: At this point, Chapter 2.1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Chapter 2.2: SLAM Principles

**Goal**: Provide a theoretical overview of SLAM and a practical tutorial for `slam_toolbox`.

**Independent Test**: Chapter content accurately explains SLAM principles; `slam_toolbox` tutorial is reproducible.

### Implementation for User Story 2

- [ ] T018 [M2] [US2] Write theoretical overview of SLAM in `book/docs/module-2-core-ai-algorithms/chapter-2-2-slam-principles.md`
- [ ] T019 [M2] [US2] Create diagrams for filter-based vs. graph-based SLAM and place in `book/static/img/module2_slam/`
- [ ] T020 [M2] [US2] Write tutorial on running and interpreting `slam_toolbox` in ROS 2 within `book/docs/module-2-core-ai-algorithms/chapter-2-2-slam-principles.md`
- [ ] T021 [M2] [US2] Develop exercises to validate understanding of SLAM using pre-recorded bag files.

**Checkpoint**: At this point, Chapter 2.2 should be fully functional and testable independently

---

## Phase 5: User Story 3 - Chapter 2.3: Reinforcement Learning

**Goal**: Introduce Reinforcement Learning concepts and provide PyTorch implementations of DQN and PPO.

**Independent Test**: Chapter content explains RL, DQN, PPO; code examples train successfully on standard environments and show positive reward trends.

### Implementation for User Story 3

- [ ] T022 [M2] [US3] Write guide to RL, DQN, and PPO in `book/docs/module-2-core-ai-algorithms/chapter-2-3-reinforcement-learning.md`
- [ ] T023 [M2] [US3] Create commented DQN agent implementation in `src/module2_examples/rl/dqn_agent.py`
- [ ] T024 [M2] [US3] Create commented PPO agent implementation in `src/module2_examples/rl/ppo_agent.py`
- [ ] T025 [M2] [US3] Create unit tests for DQN and PPO agents in `src/module2_examples/tests/test_rl.py`

**Checkpoint**: All user stories should now be independently functional

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T026 [M2] Peer-review all content in `book/docs/module-2-core-ai-algorithms/` for technical accuracy and clarity
- [ ] T027 [M2] Run all `pytest` tests for `src/module2_examples/` to ensure code examples work as described
- [ ] T028 [M2] Read through the module from a student's perspective to ensure a logical flow of concepts in `book/docs/module-2-core-ai-algorithms/`
- [ ] T029 [M2] Finalize all diagrams and format the content for Docusaurus in `book/docs/module-2-core-ai-algorithms/` and `book/static/img/module2_slam/`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Research (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Research completion.
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (Chapter 2.1 → 2.2 → 2.3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **Chapter 2.1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **Chapter 2.2 (P2)**: Can start after Foundational (Phase 2) - Independently testable
- **Chapter 2.3 (P3)**: Can start after Foundational (Phase 2) - Independently testable

### Within Each User Story

- Content writing should precede code implementation.
- Code implementation should precede testing.
- Story complete before moving to next priority.

### Parallel Opportunities

- All Research tasks can run in parallel.
- All Foundational tasks T003-T006 related to book structure and T007-T011 related to code examples can run in parallel.
- Once Foundational phase completes, User Stories (Chapter 2.1, 2.2, 2.3) can potentially start in parallel if content and code for each is handled by different contributors.
- Within Chapter 2.1: Implementation of A*, Dijkstra's, RRT (T013, T014, T015) can run in parallel before creating the ROS 2 wrapper.
- Within Chapter 2.2: Writing theoretical overview and creating diagrams (T018, T019) can run in parallel.
- Within Chapter 2.3: Implementation of DQN and PPO (T023, T024) can run in parallel.

---

## Parallel Example: User Story 1 (Chapter 2.1: Path Planning)

```bash
# Implement path planning algorithms in parallel:
Task: "Implement A* algorithm in src/module2_examples/path_planning/a_star.py"
Task: "Implement Dijkstra's algorithm in src/module2_examples/path_planning/dijkstra.py"
Task: "Implement RRT algorithm in src/module2_examples/path_planning/rrt.py"

# Later, after implementations are complete, write tests:
Task: "Create unit tests for A*, Dijkstra's, and RRT in src/module2_examples/tests/test_path_planning.py"
```

---

## Implementation Strategy

### MVP First (Chapter 2.1 Only)

1. Complete Phase 1: Research
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Chapter 2.1)
4. **STOP and VALIDATE**: Test Chapter 2.1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Research + Foundational → Foundation ready
2. Add Chapter 2.1 → Test independently → Deploy/Demo (MVP!)
3. Add Chapter 2.2 → Test independently → Deploy/Demo
4. Add Chapter 2.3 → Test independently → Deploy/Demo
5. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With multiple developers:

1. Team completes Research + Foundational together
2. Once Foundational is done:
   - Developer A: Chapter 2.1 (Path Planning)
   - Developer B: Chapter 2.2 (SLAM Principles)
   - Developer C: Chapter 2.3 (Reinforcement Learning)
3. Chapters complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence