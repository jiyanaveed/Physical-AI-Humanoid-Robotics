# Tasks: Module 2 - Core AI Algorithms for Robotics

**Input**: `.specify/memory/MODULE_2_PLAN.md`

---

## Phase 1: Research

- [X] T030 [P] [M2] Research and decide on a Python library for visualizing path planning algorithms in `research/path_planning_viz_research.md`.
- [X] T031 [P] [M2] Compile and document key resources for SLAM and RL theory in `research/slam_rl_resources.md`.

---

## Phase 2: Foundation

- [X] T032 [M2] Create the directory `book/docs/module-2-core-ai-algorithms/`.
- [X] T033 [M2] Create placeholder file `book/docs/module-2-core-ai-algorithms/chapter-2-1-path-planning.md` with Docusaurus front matter.
- [X] T034 [M2] Create placeholder file `book/docs/module-2-core-ai-algorithms/chapter-2-2-slam-principles.md` with Docusaurus front matter.
- [X] T035 [M2] Create placeholder file `book/docs/module-2-core-ai-algorithms/chapter-2-3-reinforcement-learning.md` with Docusaurus front matter.
- [X] T036 [M2] Set up Python package `src/module2_examples` for code examples with `pytest` and `pytorch` configs.

---

## Phase 3: Analysis (Content Creation)

### Chapter 2.1: Path Planning
- [ ] T037 [US1] [M2] Write the full text explaining path planning algorithms (A*, Dijkstra's, RRT) in `book/docs/module-2-core-ai-algorithms/chapter-2-1-path-planning.md`.
- [ ] T038 [P] [US1] [M2] Create a commented, from-scratch Python implementation of A* in `src/module2_examples/path_planning/a_star.py`.
- [ ] T039 [P] [US1] [M2] Create a commented, from-scratch Python implementation of Dijkstra's in `src/module2_examples/path_planning/dijkstra.py`.
- [ ] T040 [P] [US1] [M2] Create a commented, from-scratch Python implementation of RRT in `src/module2_examples/path_planning/rrt.py`.
- [ ] T041 [US1] [M2] Develop a ROS 2 wrapper node that uses the path planning algorithms in `src/module2_examples/ros/path_planner_node.py`.

### Chapter 2.2: SLAM Principles
- [ ] T042 [US2] [M2] Write the theoretical overview of SLAM in `book/docs/module-2-core-ai-algorithms/chapter-2-2-slam-principles.md`.
- [ ] T043 [US2] [M2] Create diagrams to explain filter-based vs. graph-based SLAM and add to `book/docs/module-2-core-ai-algorithms/assets/`.
- [ ] T044 [US2] [M2] Write a tutorial on how to run and interpret `slam_toolbox` in ROS 2 and add to the chapter markdown file.

### Chapter 2.3: Reinforcement Learning
- [ ] T045 [US3] [M2] Write the guide to RL, DQN, and PPO in `book/docs/module-2-core-ai-algorithms/chapter-2-3-reinforcement-learning.md`.
- [ ] T046 [P] [US3] [M2] Create a commented code example for a simple DQN agent in PyTorch in `src/module2_examples/rl/dqn_agent.py`.
- [ ] T047 [P] [US3] [M2] Create a commented code example for a simple PPO agent in PyTorch in `src/module2_examples/rl/ppo_agent.py`.

---

## Phase 4: Synthesis (Review & Polish)

- [ ] T048 [M2] Perform peer review of all content in `book/docs/module-2-core-ai-algorithms/`.
- [ ] T049 [P] [M2] Write `pytest` tests for all path planning and RL algorithms in `src/module2_examples/tests/`.
- [ ] T050 [M2] Perform a final read-through of the entire module from a student's perspective.
- [ ] T051 [M2] Finalize all diagrams and ensure they are correctly embedded in the markdown files.
- [ ] T052 [M2] **Constitution Check**: Verify all content and code in Module 2 against `.specify/memory/constitution.md`.
- [ ] T053 [M2] **Docusaurus Compliance**: Ensure all markdown files render correctly in Docusaurus.
