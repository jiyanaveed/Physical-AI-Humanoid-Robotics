<!--
Sync Impact Report:
- Version change: None -> 1.0.0
- Modified principles:
  - [PRINCIPLE_1_NAME] -> I. Academic Rigor
  - [PRINCIPLE_2_NAME] -> II. Docusaurus for Content
  - [PRINCIPLE_3_NAME] -> III. Modular FastAPI Architecture
- Added sections:
  - Technology Stack
  - Development Workflow
- Removed sections:
  - [PRINCIPLE_4_NAME]
  - [PRINCIPLE_5_NAME]
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/tasks-template.md
  - ✅ .claude/commands/speckit.constitution.md
  - ✅ .claude/commands/speckit.plan.md
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook and Platform Constitution

## Core Principles

### I. Academic Rigor
All content must be factually accurate, well-researched, and cite credible sources. All code must be well-documented and reflect best practices in research and software engineering.

### II. Docusaurus for Content
The textbook and documentation content in the `book/` and `docs/` directories must be authored and structured to be compliant with Docusaurus. This ensures a consistent, searchable, and maintainable format for the educational material.

### III. Modular FastAPI Architecture
The backend APIs, located in `rag_api/` directories, must be developed using a modular architecture based on FastAPI. This promotes separation of concerns, reusability, and scalability.

## Technology Stack

The project will primarily use the following technologies:
- **Docusaurus**: For building the textbook and documentation website.
- **FastAPI**: For creating modular and high-performance backend APIs.
- **Python**: As the primary language for the backend.

## Development Workflow

All new features or a significant changes must be developed in a separate branch and submitted as a pull request. All pull requests must be reviewed and approved by at least one other contributor before being merged.

## Governance

This Constitution is the supreme governing document of the project. All code, documentation, and contributions must adhere to its principles. Amendments to this Constitution require a proposal, review, and majority approval from the project maintainers.

**Version**: 1.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12