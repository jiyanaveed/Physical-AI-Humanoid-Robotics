# Quickstart: RAG Ingestion Pipeline

## Overview
Quick setup guide for the RAG ingestion pipeline that extracts content from Docusaurus sites, generates embeddings, and stores them in Qdrant.

## Prerequisites

### 1. System Requirements
- Python 3.8 or higher
- Git
- Access to Cohere API
- Access to Qdrant Cloud

### 2. API Keys and Configuration
- Cohere API key (for embedding generation)
- Qdrant Cloud URL and API key
- Access to Docusaurus site sitemap

## Setup

### 1. Clone and Navigate to Project
```bash
cd /Users/javerianaveed/myEbook/RAG/backend
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
# Or if using pyproject.toml:
pip install .  # from the backend directory
```

### 3. Configure Environment Variables
Create a `.env` file in the RAG directory:
```bash
cp .env.example .env
```

Edit `.env` with your credentials:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
SITEMAP_URL=https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml
COLLECTION_NAME=RAD_Embedding
```

## Usage

### 1. Run the Complete Pipeline
```bash
cd /Users/javerianaveed/myEbook/RAG/backend
python main.py
```

This executes the `ingest_book()` function which orchestrates the entire pipeline.

### 2. Run Individual Components (for testing)

#### Extract URLs from Sitemap
```python
from main import get_all_urls
urls = get_all_urls("https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml")
print(f"Found {len(urls)} URLs")
```

#### Extract Content from a Single URL
```python
from main import extract_text_from_url
text, title = extract_text_from_url("https://example.com/page")
print(f"Title: {title}")
print(f"Content length: {len(text)} characters")
```

#### Chunk Content
```python
from main import chunk_text
chunks = chunk_text(text, "https://example.com/page", title)
print(f"Created {len(chunks)} chunks")
```

#### Generate Embedding
```python
from main import embed
embedding = embed("Sample text content")
print(f"Embedding dimension: {len(embedding)}")
```

## Configuration Options

### 1. Custom Chunking Parameters
The `chunk_text` function accepts:
- `max_chars` (default: 1200): Maximum characters per chunk
- `overlap` (default: 100): Overlap between chunks in characters

### 2. Collection Name
The collection name is fixed as "RAD_Embedding" per requirements.

## Verification

### 1. Check Qdrant Collection
After running the pipeline, verify data was stored:
- Check collection "RAD_Embedding" exists in your Qdrant Cloud instance
- Verify the number of points matches expected count
- Confirm metadata fields are present

### 2. Expected Output
When running the full pipeline, you should see:
- List of discovered URLs
- Progress indicators for each page processed
- Success/failure messages for each chunk
- Final validation results

## Troubleshooting

### 1. Common Issues
- **API Rate Limits**: The pipeline includes delays to respect API limits
- **Invalid URLs**: The system skips invalid URLs and continues processing
- **Network Issues**: The system includes retry logic for transient failures

### 2. Error Messages
- "COHERE_API_KEY environment variable is required" - Check your .env file
- "QDRANT_URL and QDRANT_API_KEY environment variables are required" - Check your .env file
- "No text extracted from: [URL]" - Some pages may have content that can't be extracted

## Next Steps

1. **Customize**: Modify chunking parameters based on your content requirements
2. **Monitor**: Set up monitoring for the ingestion pipeline in production
3. **Scale**: Consider parallel processing for large sites (beyond the single-file requirement)