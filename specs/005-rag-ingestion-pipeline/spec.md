# RAG Ingestion Pipeline Feature Specification

## Objective
Build a data ingestion pipeline that extracts content from the deployed Docusaurus book, generates embeddings using Cohere, and stores them in Qdrant for downstream retrieval.

## Scope of Work
- Fetch content from deployed GitHub Pages URLs
- Parse and clean HTML/Markdown text
- Chunk content deterministically with metadata (URL, page title, section, chunk index)
- Generate embeddings via Cohere models
- Store embeddings in Qdrant Cloud (Free Tier)
- Ensure idempotent ingestion

## Target Audience
AI engineers and backend developers building a RAG system.

## Functional Requirements

### 1. Content Discovery
- System shall extract all URLs from the sitemap: https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml
- System shall identify and prioritize content pages over navigation pages

### 2. Content Extraction
- System shall extract clean text content from HTML pages
- System shall preserve metadata including page title, section information
- System shall handle various HTML structures commonly found in Docusaurus sites

### 3. Content Processing
- System shall chunk content with configurable size and overlap
- System shall preserve metadata (URL, page title, section, chunk index) with each chunk
- System shall normalize text content (remove extra whitespace, standardize formatting)

### 4. Embedding Generation
- System shall generate vector embeddings using Cohere's embedding API
- System shall handle API rate limiting and errors gracefully

### 5. Storage
- System shall store embeddings in Qdrant Cloud
- System shall use collection named "RAD_Embedding"
- System shall store complete metadata with each embedding
- System shall implement idempotent storage to prevent duplicates

### 6. Validation
- System shall validate successful storage of embeddings
- System shall verify metadata correctness in storage
- System shall provide status reporting during ingestion

## Technical Constraints
- All functionality shall be implemented in a single main.py file
- Implementation shall include specific functions: get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant, ingest_book
- Collection name shall be "RAD_Embedding"
- System shall handle errors gracefully with appropriate logging