# myEbook Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-12

## Active Technologies
- Python 3.8+ + requests, qdrant-client, cohere, beautifulsoup4, python-dotenv, trafilatura (005-rag-ingestion-pipeline)
- Qdrant Cloud vector database (005-rag-ingestion-pipeline)

- Python 3.x, ROS 2 (Humble) + NumPy + SciPy (module-4-control-dynamics)
- Docusaurus (book project)
- FastAPI (for rag_api/ if applicable)
- Node.js/npm (for Docusaurus)

## Project Structure

```text
book/
└── docs/
    └── module-04/
        ├── chapter-4-1-kinematics.md
        ├── chapter-4-2-control-theory.md
        └── chapter-4-3-advanced-dynamics.md

src/
└── module4_examples/
    ├── __init__.py
    ├── kinematics/
    │   ├── forward_kinematics.py
    │   └── inverse_kinematics.py
    ├── control/
    │   ├── pid_controller.py
    │   └── trajectory_generation.py
    ├── dynamics/
    │   └── non_linear_control.py
    └── tests/
        ├── test_kinematics.py
        ├── test_control.py
        └── test_dynamics.py

# Overall Project Structure
.specify/
book/
src/
rag_api/ # if it exists
```

## Commands

- Python (for `src/` code): `colcon build`, `colcon test`, `ruff check .`
- Docusaurus (for `book/`): `npm install`, `npm start`

## Code Style

- Python: PEP 8, Black formatter, Ruff linter.
- Docusaurus (TypeScript/React): ESLint, Prettier.

## Recent Changes
- 005-rag-ingestion-pipeline: Added Python 3.8+ + requests, qdrant-client, cohere, beautifulsoup4, python-dotenv, trafilatura

- module-4-control-dynamics: Added Python 3.x, ROS 2 (Humble) + NumPy + SciPy, Docusaurus

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
