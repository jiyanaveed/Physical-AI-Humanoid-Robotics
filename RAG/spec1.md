# RAG Content Ingestion Pipeline Specification

## Overview
This document specifies the architecture and implementation details for a data ingestion pipeline that extracts content from a deployed Docusaurus book, generates embeddings using Cohere, and stores them in Qdrant for downstream retrieval.

## Objective
Build a data ingestion pipeline that:
- Fetches content from deployed GitHub Pages URLs
- Parses and cleans HTML/Markdown text
- Chunks content deterministically with metadata (URL, page title, section, chunk index)
- Generates embeddings via Cohere models
- Stores embeddings in Qdrant Cloud (Free Tier)
- Ensures idempotent ingestion

## Target Audience
AI engineers and backend developers building a RAG system.

## Architecture

### Components

#### 1. URL Extraction Module
- **Purpose**: Extract all page URLs from the Docusaurus sitemap
- **Input**: Sitemap URL (e.g., `https://physicalhumanoidaitextbook.vercel.app/sitemap.xml`)
- **Output**: List of page URLs to process
- **Technology**: `xml.etree.ElementTree` for XML parsing
- **Implementation**: `get_all_urls()` function

#### 2. Content Extraction Module
- **Purpose**: Download web pages and extract clean text content with metadata
- **Input**: Page URL
- **Output**: Tuple of (clean text, page title)
- **Technology**: `requests` for HTTP requests, `BeautifulSoup4` for HTML parsing
- **Implementation**: `extract_text_from_url()` function
- **Features**:
  - Removes navigation, headers, footers, scripts, and styles
  - Extracts page title from HTML
  - Cleans up excessive whitespace
  - Fallback to `trafilatura` if BeautifulSoup extraction fails

#### 3. Content Chunking Module
- **Purpose**: Split content into overlapping chunks with metadata preservation
- **Input**: Text content, URL, page title
- **Output**: List of tuples (chunk_text, chunk_index, page_title, url)
- **Technology**: Custom chunking algorithm
- **Implementation**: `chunk_text()` function
- **Features**:
  - Configurable chunk size (default 1200 characters)
  - Configurable overlap (default 100 characters)
  - Sentence-boundary aware splitting
  - Word-boundary fallback for splitting
  - Prevention of infinite loops

#### 4. Embedding Generation Module
- **Purpose**: Generate vector embeddings for text chunks
- **Input**: Text chunk
- **Output**: Embedding vector (1024 dimensions for Cohere embed-english-v3.0)
- **Technology**: Cohere API
- **Implementation**: `embed()` function
- **Model**: `embed-english-v3.0`
- **Input type**: `search_document` for document chunks

#### 5. Storage Module
- **Purpose**: Store embeddings in Qdrant Cloud with rich metadata
- **Input**: Chunk text, embedding vector, metadata
- **Output**: Stored vector in Qdrant collection
- **Technology**: Qdrant Cloud client
- **Implementation**: `save_chunk_to_qdrant()` function
- **Metadata stored**:
  - URL
  - Page title
  - Chunk text
  - Chunk ID
  - Chunk index
  - Source URL
  - Chunk hash (for duplicate detection)

#### 6. Idempotency Module
- **Purpose**: Prevent duplicate processing of the same content
- **Input**: URL, chunk index, chunk length
- **Output**: Boolean indicating if already processed
- **Technology**: Qdrant query functionality
- **Implementation**: `check_if_already_ingested()` function
- **Features**:
  - Uses URL and chunk_index combination to detect duplicates
  - Hash-based duplicate detection

### Configuration

#### Environment Variables
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `SITEMAP_URL`: URL to sitemap (default: deployed site)
- `COLLECTION_NAME`: Name of Qdrant collection (default: humanoid_ai_book)

#### Default Values
- Collection: `humanoid_ai_book`
- Embedding model: `embed-english-v3.0`
- Chunk size: 1200 characters
- Overlap: 100 characters
- Vector dimensions: 1024
- Distance metric: Cosine

## Data Flow

1. **URL Extraction**:
   - Fetch sitemap XML from `SITEMAP_URL`
   - Parse XML to extract all page URLs
   - Return list of URLs to process

2. **Content Processing Loop**:
   For each URL:
   a. Download page content
   b. Extract clean text and page title
   c. Skip if no content extracted
   d. Check if already processed (idempotency)
   e. Skip if already processed
   f. Chunk the content with metadata
   g. For each chunk:
      - Generate embedding
      - Store in Qdrant with metadata
      - Increment global ID

3. **Storage**:
   - Create Qdrant collection if not exists
   - Store each chunk as a point with vector and metadata
   - Use chunk ID as the point ID

## Metadata Schema

Each stored point contains the following payload:
```json
{
  "url": "https://example.com/page",
  "page_title": "Page Title",
  "text": "Chunked text content...",
  "chunk_id": 123,
  "chunk_index": 0,
  "source_url": "https://example.com/page",
  "chunk_hash": "md5_hash_value"
}
```

## Idempotency Implementation

The system implements idempotent ingestion through:
1. **Duplicate Detection**: Query Qdrant for existing chunks with same URL and chunk_index
2. **Hash-Based Verification**: Generate content hash to detect if content has changed
3. **Conditional Processing**: Skip processing if chunk already exists

## Error Handling

- HTTP request failures are raised as exceptions
- Missing environment variables raise `ValueError`
- Failed content extraction logs warnings but continues processing
- Failed duplicate checks default to processing (safe behavior)

## Dependencies

- `requests`: HTTP requests
- `qdrant-client`: Qdrant Cloud interaction
- `cohere`: Embedding generation
- `beautifulsoup4`: HTML parsing
- `python-dotenv`: Environment variable management
- `trafilatura`: Fallback text extraction
- `xml.etree.ElementTree`: Sitemap XML parsing

## Testing

The implementation includes:
- Unit tests for each module
- Integration tests for the complete pipeline
- Validation of URL extraction
- Content extraction validation
- Chunking algorithm verification
- Idempotency check functionality

## Deployment

1. Install dependencies: `pip install -r requirements.txt`
2. Configure environment: `cp .env.example .env` and add credentials
3. Run ingestion: `python main.py`

## Security Considerations

- API keys stored in environment variables (not hardcoded)
- Input validation for URLs and configuration
- Secure HTTP connections for all API calls
- Proper error handling to prevent information leakage

## Performance Considerations

- Overlapping chunks to preserve context across boundaries
- Efficient XML parsing for large sitemaps
- Batch processing considerations for Qdrant storage
- Memory-efficient text processing for large documents

## Scalability

- Modular design allows for parallel processing
- Configurable chunk sizes for different content types
- Efficient duplicate detection to prevent reprocessing
- Qdrant Cloud provides horizontal scaling