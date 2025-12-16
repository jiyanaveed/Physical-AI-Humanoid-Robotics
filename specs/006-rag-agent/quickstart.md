# Quickstart: RAG Agent Development

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Access to OpenAI API
- Access to Qdrant Cloud (with API key and URL)

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
pip install fastapi openai qdrant-client python-dotenv uvicorn cohere
```

### 4. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
PORT=8000
```

### 5. Project Structure
Create the following directory structure:
```
rag_api/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
├── main.py
├── requirements.txt
└── .env
```

## Running the Application

### 1. Start the Server
```bash
cd rag_api
uvicorn main:app --reload --port 8000
```

### 2. Verify Installation
Open your browser or use curl to test the server:
```bash
curl http://localhost:8000/health
```

You should receive a response confirming the server is running.

## Testing the RAG Agent

### 1. Send a Query
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Your question about the book content"
  }'
```

### 2. Expected Response
The API should return relevant book content chunks with metadata:
```json
{
  "query_id": "unique-id",
  "results": [
    {
      "id": "chunk-id",
      "text": "Relevant content text...",
      "url": "source-url",
      "page_title": "Page Title",
      "section": "Section Name",
      "chunk_index": 1,
      "source_url": "original-source-url",
      "chunk_hash": "content-hash",
      "score": 0.95
    }
  ],
  "total_results": 1,
  "processing_time": 123.4,
  "status": "success"
}
```

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify QDRANT_URL and QDRANT_API_KEY in your .env file
2. **API Key Errors**: Ensure OPENAI_API_KEY is set correctly
3. **No Results**: Check that the RAD_Embedding collection exists in Qdrant with content
4. **Slow Response**: Consider reducing the top_k parameter for faster responses

### Verification Steps
1. Confirm Qdrant collection exists: `curl -X GET {QDRANT_URL}/collections/RAD_Embedding`
2. Test OpenAI API access: Simple test call to OpenAI API
3. Check environment variables: `echo $OPENAI_API_KEY` (should not be empty)