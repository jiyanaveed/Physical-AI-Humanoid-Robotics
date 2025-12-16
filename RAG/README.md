# RAG Content Ingestion and Retrieval Testing Pipeline

This project implements a RAG (Retrieval-Augmented Generation) system with two main components:
1. An ingestion pipeline that extracts content from the deployed Docusaurus book, generates embeddings using Cohere, and stores them in Qdrant
2. A comprehensive testing system for validating the data retrieval pipeline

## Features

- Fetches content from deployed GitHub Pages URLs via sitemap
- Parses and cleans HTML content with metadata extraction
- Chunks content deterministically with overlap and metadata (URL, page title, section, chunk index)
- Generates embeddings via Cohere models
- Stores embeddings in Qdrant Cloud with duplicate prevention
- Idempotent ingestion to avoid processing duplicates

## Prerequisites

- Python 3.8+
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Copy the environment file and add your credentials:
```bash
cp .env.example .env
```

3. Edit `.env` and add your API keys:
```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Usage

Run the ingestion pipeline:
```bash
python main.py
```

## Configuration

- `SITEMAP_URL`: URL to the sitemap of your Docusaurus site (default: deployed site)
- `COLLECTION_NAME`: Name of the Qdrant collection (default: humanoid_ai_book)

## Testing

### Ingestion Pipeline Testing
Run the ingestion pipeline validation:
```bash
python main.py
```

### Retrieval Pipeline Testing
Run the comprehensive retrieval testing system to validate that embeddings can be properly retrieved:
```bash
python retrieval_test.py
```

The retrieval testing system includes:
- Connection to Qdrant collection verification
- Sample text and keyword-based query execution
- Metadata integrity validation (URL, page title, chunk index, etc.)
- Edge case handling (empty queries, no results, duplicates)
- Performance metrics and logging
- Comprehensive reporting

## Pipeline Details

1. **URL Extraction**: Extracts all page URLs from the sitemap
2. **Content Extraction**: Downloads each page and extracts clean text content with page title
3. **Content Chunking**: Splits content into overlapping chunks with metadata preservation
4. **Embedding Generation**: Creates vector embeddings using Cohere's embed-english-v3.0 model
5. **Storage**: Stores embeddings in Qdrant with rich metadata
6. **Duplicate Prevention**: Checks for existing chunks to prevent reprocessing

## Retrieval Testing System Architecture

The retrieval testing system follows a modular design:

- `retrieval_test.py`: Main entry point for running the complete testing pipeline
- `test_retrieval/`: Module containing specialized functionality:
  - `connection.py`: Qdrant connection utilities and validation
  - `queries.py`: Query execution functions for text and keyword-based searches
  - `validation.py`: Metadata and content validation functions
  - `logging.py`: Logging, reporting, and performance metrics utilities