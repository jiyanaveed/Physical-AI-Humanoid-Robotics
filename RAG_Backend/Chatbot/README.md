# RAG Agent API

API for the RAG (Retrieval-Augmented Generation) agent that queries Qdrant embeddings and returns relevant book content.

## Overview

This API provides endpoints for querying a RAG system that retrieves relevant book content based on user questions. It integrates with OpenAI Agent SDK, Qdrant vector database, and Cohere for embeddings.

## Prerequisites

- Python 3.11+
- pip package manager
- Access to OpenAI API
- Access to Qdrant Cloud (with API key and URL)
- Access to Cohere API

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the rag_api directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
PORT=8000
COLLECTION_NAME=RAD_Embedding
LOG_LEVEL=INFO
```

## Running the Application

### 1. Start the Server
```bash
cd rag_api
python main.py
```

Or using uvicorn directly:
```bash
uvicorn rag_api.main:app --reload --port 8000
```

### 2. Verify Installation
Open your browser or use curl to test the server:
```bash
curl http://localhost:8000/health
```

You should receive a response confirming the server is running.

## API Usage

### Query Endpoint
Send a query to get relevant book content:

```bash
curl -X POST http://localhost:8000/query/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Your question about the book content",
    "top_k": 5
  }'
```

### Test Endpoint
Use the test endpoint to verify the API structure without calling external services:

```bash
curl -X POST http://localhost:8000/query/test \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Test question",
    "top_k": 3
  }'
```

## API Endpoints

### Health Check
- `GET /health/` - Check if the API is operational

### Query
- `POST /query/` - Submit a question and get relevant content
- `POST /query/test` - Test endpoint with mock responses

## Project Structure

```
rag_api/
├── main.py              # FastAPI application entry point
├── src/
│   ├── models/          # Data models (UserQuery, ContentChunk, etc.)
│   │   ├── query.py
│   │   ├── content_chunk.py
│   │   └── response.py
│   ├── services/        # Business logic (Qdrant integration, agent logic)
│   │   ├── qdrant_service.py
│   │   ├── agent_service.py
│   │   └── logging_service.py
│   ├── api/             # API endpoints
│   │   ├── query_router.py
│   │   └── health_router.py
│   └── config.py        # Configuration management
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
└── .env.example
```

## Configuration

The application can be configured using environment variables:

- `OPENAI_API_KEY` - API key for OpenAI services
- `QDRANT_URL` - URL for Qdrant Cloud instance
- `QDRANT_API_KEY` - API key for Qdrant Cloud
- `COHERE_API_KEY` - API key for Cohere services
- `PORT` - Port to run the server on (default: 8000)
- `COLLECTION_NAME` - Name of the Qdrant collection (default: RAD_Embedding)
- `LOG_LEVEL` - Logging level (default: INFO)

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify QDRANT_URL and QDRANT_API_KEY in your .env file
2. **API Key Errors**: Ensure OPENAI_API_KEY and COHERE_API_KEY are set correctly
3. **No Results**: Check that the RAD_Embedding collection exists in Qdrant with content
4. **Slow Response**: Consider reducing the top_k parameter for faster responses

### Verification Steps
1. Confirm Qdrant collection exists: `curl -X GET {QDRANT_URL}/collections/RAD_Embedding`
2. Test OpenAI API access: Simple test call to OpenAI API
3. Check environment variables: `echo $OPENAI_API_KEY` (should not be empty)