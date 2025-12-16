# Data Model: RAG Agent Development

## Key Entities

### User Query
**Description**: A text-based question submitted by the user that requires relevant book content as a response
**Fields**:
- `query_text` (string): The actual question text from the user
- `query_id` (string): Unique identifier for the query
- `timestamp` (datetime): When the query was submitted
- `user_id` (string, optional): Identifier for the user (if implemented later)

**Validation Rules**:
- `query_text` must not be empty
- `query_text` length should be reasonable (e.g., max 1000 characters)

### Content Chunk
**Description**: A segment of book content retrieved from Qdrant that matches the user's query, including the text content and associated metadata
**Fields**:
- `id` (string): Unique identifier for the chunk
- `text` (string): The actual content text
- `url` (string): URL of the source page
- `page_title` (string): Title of the source page
- `section` (string): Section name in the source document
- `chunk_index` (integer): Index position of this chunk in the original content
- `source_url` (string): URL of the original source
- `chunk_hash` (string): Hash of the content for deduplication
- `score` (float): Relevance score from Qdrant search

**Validation Rules**:
- `text` must not be empty
- `url` must be a valid URL format
- `chunk_index` must be non-negative

### API Response
**Description**: The structured output returned to the client, containing relevant content chunks and metadata
**Fields**:
- `query_id` (string): Identifier linking to the original query
- `results` (array of ContentChunk): Array of relevant content chunks
- `total_results` (integer): Total number of results returned
- `processing_time` (float): Time taken to process the query in milliseconds
- `status` (string): Status of the request (success, error)
- `error_message` (string, optional): Error details if status is error

**Validation Rules**:
- `results` array must contain valid ContentChunk objects
- `total_results` must match the actual number of results in the array

### Query Processing Log
**Description**: A record of user queries, system responses, processing time, and any errors encountered
**Fields**:
- `log_id` (string): Unique identifier for the log entry
- `query_text` (string): The original query text
- `response_summary` (string): Brief summary of the response
- `processing_time` (float): Time taken to process in milliseconds
- `timestamp` (datetime): When the query was processed
- `status` (string): Status of processing (success, error)
- `error_details` (string, optional): Details if an error occurred

**Validation Rules**:
- `timestamp` must be current or past
- `processing_time` must be non-negative

## Relationships

### User Query → Content Chunk
- One query can result in multiple content chunks (one-to-many)
- Relationship: A single user query may return multiple relevant content chunks

### API Response → Content Chunk
- One API response contains multiple content chunks (one-to-many)
- Relationship: Each API response aggregates multiple content chunks

### Query Processing Log → User Query
- One log entry per query processed (one-to-one)
- Relationship: Each processed query creates one log entry

## State Transitions

### Query Processing States
1. **Received**: Query has been received by the API
2. **Processing**: Query is being processed by the agent
3. **Retrieved**: Relevant content has been retrieved from Qdrant
4. **Formatted**: Response is being formatted
5. **Completed**: Response has been sent to the client
6. **Failed**: An error occurred during processing

## API Contract

### Query Endpoint
```
POST /query
Content-Type: application/json

Request Body:
{
  "query": "string (required) - The user question",
  "top_k": "integer (optional) - Number of results to return, default 5"
}

Response:
{
  "query_id": "string - Unique identifier for the query",
  "results": [
    {
      "id": "string - Chunk identifier",
      "text": "string - Content text",
      "url": "string - Source URL",
      "page_title": "string - Page title",
      "section": "string - Section name",
      "chunk_index": "integer - Position in source",
      "source_url": "string - Original source URL",
      "chunk_hash": "string - Content hash",
      "score": "float - Relevance score"
    }
  ],
  "total_results": "integer - Number of results returned",
  "processing_time": "float - Time in milliseconds",
  "status": "string - success or error"
}
```

### Error Response
```
{
  "query_id": "string - Unique identifier for the query",
  "status": "string - error",
  "error_message": "string - Description of the error",
  "processing_time": "float - Time in milliseconds"
}
```