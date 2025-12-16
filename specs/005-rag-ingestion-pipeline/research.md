# Research: RAG Ingestion Pipeline

## Overview
Research for implementing a RAG ingestion pipeline that extracts content from Docusaurus GitHub Pages, chunks it with metadata, generates Cohere embeddings, and stores them in Qdrant Cloud.

## Technology Research

### 1. Content Extraction from Docusaurus Sites
**Decision**: Use BeautifulSoup4 with requests for HTML parsing, with trafilatura as fallback
**Rationale**: BeautifulSoup provides fine-grained control over HTML parsing and allows for metadata extraction, while trafilatura provides robust fallback for complex HTML structures.
**Alternatives considered**:
- Selenium (for JS-heavy sites) - rejected due to performance overhead
- Scrapy (for complex crawling) - rejected as overkill for sitemap-based extraction

### 2. Sitemap Processing
**Decision**: Use xml.etree.ElementTree for parsing sitemap XML
**Rationale**: Built-in Python library, efficient for XML parsing, perfect for sitemap processing
**Alternatives considered**:
- lxml (faster) - rejected as stdlib solution is sufficient
- Third-party XML libraries - rejected to minimize dependencies

### 3. Text Chunking Strategy
**Decision**: Implement overlapping window chunking with sentence-boundary awareness
**Rationale**: Preserves context across chunks while maintaining semantic boundaries
**Alternatives considered**:
- Fixed character splitting (simpler but breaks context) - rejected
- Semantic chunking (more sophisticated but complex) - rejected for MVP

### 4. Embedding Generation
**Decision**: Use Cohere's embed-english-v3.0 model via Cohere Python SDK
**Rationale**: High-quality embeddings, good for academic content, well-documented API
**Alternatives considered**:
- OpenAI embeddings (cost concerns) - rejected for free tier usage
- Local models (privacy but resource intensive) - rejected for simplicity

### 5. Vector Storage
**Decision**: Use Qdrant Cloud with cosine distance metric
**Rationale**: Free tier available, good performance, Python SDK available, cosine distance standard for embeddings
**Alternatives considered**:
- Pinecone (cost concerns) - rejected for budget reasons
- ChromaDB (self-hosted) - rejected for maintenance simplicity
- Weaviate (alternative cloud option) - rejected as Qdrant meets requirements

### 6. Idempotency Implementation
**Decision**: Use content hash combined with URL and chunk index for duplicate detection
**Rationale**: Ensures no duplicate processing while allowing updates to content
**Alternatives considered**:
- Simple URL tracking (less robust) - rejected
- Database of processed items (more complex) - rejected for simplicity

## Architecture Decisions

### Single File Implementation
**Decision**: Implement all functionality in single main.py file as required
**Rationale**: Meets specific requirement while maintaining modularity through function separation
**Implications**: Code organization through clear function boundaries and documentation

### Required Functions Design
**Decision**: Implement the 7 required functions with specific signatures:
- get_all_urls: Extract URLs from sitemap
- extract_text_from_url: Parse HTML and extract content with metadata
- chunk_text: Split content with metadata preservation
- embed: Generate Cohere embeddings
- create_collection: Set up Qdrant collection named "RAD_Embedding"
- save_chunk_to_qdrant: Store embeddings with metadata
- ingest_book: Main orchestration function

## Error Handling Strategy
**Decision**: Comprehensive error handling with graceful degradation
**Rationale**: Ensures pipeline continues processing even when individual pages fail
**Implementation**: Try-catch blocks around all external operations with appropriate logging

## Performance Considerations
**Decision**: Process pages sequentially with rate limiting to respect API limits
**Rationale**: Balances processing speed with API usage constraints and server load
**Implementation**: Add delays between API calls and implement retry logic for transient failures