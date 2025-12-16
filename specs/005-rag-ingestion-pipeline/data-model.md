# Data Model: RAG Ingestion Pipeline

## Overview
Data models for the RAG ingestion pipeline that processes Docusaurus content and stores embeddings in Qdrant.

## Core Entities

### 1. ContentChunk
**Description**: Represents a processed chunk of content with metadata

**Fields**:
- `id` (int): Sequential ID for the chunk in the processing pipeline
- `text` (str): The actual text content of the chunk
- `url` (str): Original URL where the content was found
- `page_title` (str): Title of the source page
- `chunk_index` (int): Index of this chunk within the original page
- `chunk_hash` (str): MD5 hash for duplicate detection
- `embedding` (list[float]): Vector embedding (1024 dimensions for Cohere embed-english-v3.0)
- `source_url` (str): Redundant copy of URL for consistency

**Validation Rules**:
- `text` must not be empty
- `url` must be a valid URL format
- `chunk_index` must be non-negative
- `embedding` must have exactly 1024 elements (for Cohere model)

### 2. PageContent
**Description**: Represents a complete page extracted from the Docusaurus site

**Fields**:
- `url` (str): The page URL
- `title` (str): Page title extracted from HTML
- `raw_content` (str): Raw HTML content before processing
- `clean_content` (str): Clean text content after processing
- `chunks` (list[ContentChunk]): List of chunks created from this page
- `processed_at` (datetime): Timestamp when processing was completed

**Validation Rules**:
- `url` must be a valid URL
- `clean_content` must not be empty if page was successfully processed
- `chunks` list must not be empty after processing

### 3. ProcessingJob
**Description**: Represents a complete ingestion job

**Fields**:
- `id` (str): Unique identifier for the job
- `sitemap_url` (str): URL of the sitemap being processed
- `start_time` (datetime): When the job started
- `end_time` (datetime): When the job ended
- `status` (str): Current status (pending, running, completed, failed)
- `total_pages` (int): Total number of pages to process
- `processed_pages` (int): Number of pages successfully processed
- `failed_pages` (int): Number of pages that failed processing
- `total_chunks` (int): Total number of chunks created
- `error_log` (list[str]): List of errors encountered during processing

**Validation Rules**:
- `status` must be one of the allowed values
- `total_pages` >= `processed_pages` + `failed_pages`
- `sitemap_url` must be a valid URL

## Qdrant Collection Schema

### RAD_Embedding Collection
**Vector Configuration**:
- Size: 1024 (Cohere embed-english-v3.0)
- Distance: Cosine

**Payload Fields**:
- `url` (keyword): Original source URL
- `page_title` (text): Title of the source page
- `text` (text): The chunk text content
- `chunk_id` (integer): Sequential chunk identifier
- `chunk_index` (integer): Index within the original page
- `source_url` (keyword): Redundant URL field
- `chunk_hash` (keyword): MD5 hash for duplicate detection

## State Transitions

### ContentChunk States
- Created: Chunk is initialized after text processing
- Embedded: Embedding has been generated
- Stored: Chunk has been saved to Qdrant
- Validated: Chunk has been verified in storage

### ProcessingJob States
- Pending: Job is initialized but not started
- Running: Job is actively processing pages
- Completed: All pages processed successfully
- Failed: Job encountered unrecoverable errors
- Partial: Some pages processed, some failed

## Relationships

1. One `ProcessingJob` contains many `PageContent` entities
2. One `PageContent` produces many `ContentChunk` entities
3. Many `ContentChunk` entities are stored in one Qdrant collection (RAD_Embedding)