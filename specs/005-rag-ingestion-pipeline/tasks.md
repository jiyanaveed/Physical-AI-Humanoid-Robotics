# Implementation Tasks: RAG Ingestion Pipeline

**Feature**: RAG Ingestion Pipeline
**Branch**: `005-rag-ingestion-pipeline`
**Date**: 2025-12-17
**Spec**: `/specs/005-rag-ingestion-pipeline/spec.md`

## Overview
Implementation of a RAG ingestion pipeline that extracts content from Docusaurus GitHub Pages, chunks it with metadata, generates Cohere embeddings, and stores them in Qdrant Cloud. All functionality implemented in a single main.py file with specific required functions.

## Dependencies
- Python 3.8+
- Cohere API access
- Qdrant Cloud access
- Target site: https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/
- Sitemap: https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml

## Parallel Execution Examples
- T002 [P], T003 [P], T004 [P] - Set up different dependencies in parallel
- T015 [P] [US1], T016 [P] [US1] - Implement different utility functions in parallel
- T025 [P] [US2], T026 [P] [US2] - Set up different embedding and storage functions in parallel

## Implementation Strategy
1. **MVP**: Implement basic sitemap extraction and content parsing (US1)
2. **Incremental**: Add chunking and metadata preservation (US2)
3. **Complete**: Add embedding generation and Qdrant storage (US3)
4. **Validation**: Add idempotency and validation features (US4)

---

## Phase 1: Setup

### Goal
Initialize project structure and dependencies for the RAG ingestion pipeline.

### Tasks

- [X] T001 Create backend directory structure in /Users/javerianaveed/myEbook/RAG/backend
- [X] T002 [P] Install requests dependency in backend/pyproject.toml
- [X] T003 [P] Install qdrant-client dependency in backend/pyproject.toml
- [X] T004 [P] Install cohere dependency in backend/pyproject.toml
- [X] T005 [P] Install beautifulsoup4 dependency in backend/pyproject.toml
- [X] T006 [P] Install python-dotenv dependency in backend/pyproject.toml
- [X] T007 [P] Install trafilatura dependency in backend/pyproject.toml
- [X] T008 Create requirements.txt file with all dependencies
- [X] T009 Create main.py file in backend directory
- [X] T010 Add proper imports to main.py for all required libraries

---

## Phase 2: Foundational

### Goal
Set up configuration and basic infrastructure needed for all user stories.

### Tasks

- [X] T011 Configure environment variables loading in main.py
- [X] T012 Set up Cohere client with API key from environment
- [X] T013 Set up Qdrant client with URL and API key from environment
- [X] T014 Define constants for collection name (RAD_Embedding) and embedding model
- [X] T015 Create configuration validation function to check required environment variables
- [X] T016 Set up error handling utilities for graceful degradation
- [X] T017 Create logging utilities for tracking pipeline progress

---

## Phase 3: [US1] Content Discovery

### Goal
Extract all URLs from the sitemap and identify content pages over navigation pages.

### Independent Test Criteria
- Can successfully extract URLs from the sitemap
- Can identify at least one content page from the extracted URLs
- Function handles malformed XML gracefully

### Tasks

- [X] T018 [US1] Implement get_all_urls function to extract URLs from sitemap
- [X] T019 [US1] Add XML parsing with error handling for malformed sitemaps
- [X] T020 [US1] Add support for both regular sitemap and sitemap index formats
- [X] T021 [US1] Implement URL filtering to prioritize content pages over navigation
- [X] T022 [US1] Add logging of discovered URLs for debugging
- [X] T023 [US1] Test sitemap extraction with the target site
- [X] T024 [US1] Validate that extracted URLs are properly formatted

---

## Phase 4: [US2] Content Extraction

### Goal
Extract clean text content from HTML pages while preserving metadata including page title and section information.

### Independent Test Criteria
- Can extract clean text from a sample HTML page
- Can extract page title from HTML
- Handles various HTML structures found in Docusaurus sites
- Preserves metadata in extracted content

### Tasks

- [X] T025 [US2] Implement extract_text_from_url function with BeautifulSoup
- [X] T026 [US2] Add page title extraction from HTML
- [X] T027 [US2] Remove navigation, headers, footers, scripts, and styles from content
- [X] T028 [US2] Implement fallback to trafilatura for complex HTML structures
- [X] T029 [US2] Add text normalization (remove extra whitespace, standardize formatting)
- [X] T030 [US2] Test content extraction on multiple Docusaurus page types
- [X] T031 [US2] Validate metadata preservation (URL, page title)
- [X] T032 [US2] Add error handling for network and parsing failures

---

## Phase 5: [US3] Content Processing

### Goal
Chunk content with configurable size and overlap while preserving metadata and normalizing text content.

### Independent Test Criteria
- Can chunk content with specified size and overlap parameters
- Preserves all metadata (URL, page title, section, chunk index) with each chunk
- Normalizes text content properly
- Handles edge cases (very short content, content that fits in single chunk)

### Tasks

- [X] T033 [US3] Implement chunk_text function with configurable size and overlap
- [X] T034 [US3] Add sentence-boundary aware chunking to preserve context
- [X] T035 [US3] Add word-boundary fallback for chunking
- [X] T036 [US3] Preserve metadata (URL, page title, chunk index) with each chunk
- [X] T037 [US3] Add text normalization within chunks (remove extra whitespace)
- [X] T038 [US3] Handle edge cases: content shorter than max_chars, empty content
- [X] T039 [US3] Prevent infinite loops in chunking algorithm
- [X] T040 [US3] Test chunking with various content sizes and structures
- [X] T041 [US3] Validate metadata preservation across all chunks

---

## Phase 6: [US4] Embedding Generation

### Goal
Generate vector embeddings using Cohere's embedding API and handle rate limiting and errors gracefully.

### Independent Test Criteria
- Can generate embeddings for text chunks using Cohere API
- Handles API rate limiting gracefully
- Handles API errors with appropriate fallbacks
- Embeddings have correct dimensions (1024 for Cohere embed-english-v3.0)

### Tasks

- [X] T042 [US4] Implement embed function using Cohere's embed-english-v3.0 model
- [X] T043 [US4] Set appropriate input_type for document chunks ('search_document')
- [X] T044 [US4] Add error handling for Cohere API failures
- [X] T045 [US4] Implement rate limiting to respect API constraints
- [X] T046 [US4] Add retry logic for transient API failures
- [X] T047 [US4] Validate embedding dimensions (should be 1024)
- [X] T048 [US4] Test embedding generation with sample content
- [X] T049 [US4] Add logging for embedding generation performance

---

## Phase 7: [US5] Storage

### Goal
Store embeddings in Qdrant Cloud with complete metadata and implement idempotent storage to prevent duplicates.

### Independent Test Criteria
- Can create Qdrant collection named "RAD_Embedding"
- Can store embeddings with complete metadata in Qdrant
- Implements idempotent storage to prevent duplicate entries
- Successfully handles storage errors gracefully

### Tasks

- [X] T050 [US5] Implement create_collection function for RAD_Embedding collection
- [X] T051 [US5] Configure Qdrant collection with 1024-dim vectors and cosine distance
- [X] T052 [US5] Implement save_chunk_to_qdrant function with metadata storage
- [X] T053 [US5] Add complete metadata to Qdrant payload (URL, page_title, text, chunk_id, chunk_index, source_url, chunk_hash)
- [X] T054 [US5] Implement idempotency using content hash for duplicate detection
- [X] T055 [US5] Add error handling for Qdrant storage failures
- [X] T056 [US5] Test storage with sample embeddings and metadata
- [X] T057 [US5] Validate idempotency by attempting to store duplicate content
- [X] T058 [US5] Verify collection schema matches data model requirements

---

## Phase 8: [US6] Validation

### Goal
Validate successful storage of embeddings, verify metadata correctness, and provide status reporting during ingestion.

### Independent Test Criteria
- Can validate that embeddings were successfully stored in Qdrant
- Can verify metadata correctness in storage
- Provides status reporting during ingestion process
- Can detect and report validation failures

### Tasks

- [X] T059 [US6] Implement validate_pipeline function to check Qdrant vectors
- [X] T060 [US6] Add collection info validation (points count, vector size, distance)
- [X] T061 [US6] Verify metadata fields exist in stored points (url, page_title, text, etc.)
- [X] T062 [US6] Validate vector dimensions are correct (1024 for Cohere embeddings)
- [X] T063 [US6] Add sample point validation to check metadata correctness
- [X] T064 [US6] Implement status reporting during ingestion process
- [X] T065 [US6] Add progress tracking for pages and chunks processed
- [X] T066 [US6] Create validation summary with success/failure metrics
- [X] T067 [US6] Test validation against stored embeddings

---

## Phase 9: [US7] Main Pipeline Integration

### Goal
Orchestrate the complete pipeline with the ingest_book function that executes all components in sequence.

### Independent Test Criteria
- Can execute the complete pipeline from sitemap extraction to validation
- Handles errors gracefully throughout the pipeline
- Reports comprehensive status at the end
- Maintains idempotency across the entire pipeline

### Tasks

- [X] T068 [US7] Implement ingest_book function as main orchestration function
- [X] T069 [US7] Integrate sitemap extraction with content processing
- [X] T070 [US7] Connect content processing to embedding generation
- [X] T071 [US7] Connect embedding generation to Qdrant storage
- [X] T072 [US7] Add comprehensive error handling throughout the pipeline
- [X] T073 [US7] Implement progress tracking and reporting
- [X] T074 [US7] Add idempotency checks at chunk level
- [X] T075 [US7] Integrate validation at the end of the pipeline
- [X] T076 [US7] Test complete end-to-end pipeline with target site
- [X] T077 [US7] Validate that all requirements from spec are met

---

## Phase 10: Polish & Cross-Cutting Concerns

### Goal
Finalize implementation with documentation, testing, and optimization.

### Tasks

- [X] T078 Add comprehensive docstrings to all functions
- [X] T079 Update README with usage instructions for the new pipeline
- [X] T080 Add configuration documentation for environment variables
- [X] T081 Create usage examples for each function
- [X] T082 Optimize performance for large sitemaps
- [X] T083 Add additional error handling for edge cases
- [X] T084 Test pipeline with the complete target site
- [X] T085 Update .env.example with proper defaults and documentation
- [X] T086 Final validation of all requirements from the original specification
- [X] T087 Clean up temporary files and add proper file headers
- [X] T088 Update agent.py to use the new pipeline if applicable
- [X] T089 Document any changes made to existing infrastructure