# Data Model: Frontend Chatbot Integration

## Key Entities

### User Query
**Description**: A text-based question submitted by the user through the chatbot interface, which may be scoped to selected text on the current page
**Fields**:
- `query_text` (string): The actual question text from the user
- `selected_text` (string, optional): Text that was selected on the current page when the query was made
- `query_id` (string): Unique identifier for the query
- `timestamp` (datetime): When the query was submitted
- `page_context` (object): Information about the current page where the query originated
  - `url` (string): Current page URL
  - `title` (string): Current page title
  - `section` (string): Current section name

**Validation Rules**:
- `query_text` must not be empty
- `query_text` length should be reasonable (e.g., max 1000 characters)
- `selected_text` if provided should be a subset of the current page content

### Chatbot Response
**Description**: The structured answer returned from the RAG system, including the response text and source attribution metadata
**Fields**:
- `response_id` (string): Unique identifier for the response
- `query_id` (string): Reference to the original query
- `answer_text` (string): The main answer text from the RAG system
- `source_chunks` (array of SourceChunk): Array of content chunks that support the answer
- `timestamp` (datetime): When the response was received
- `status` (string): Status of the response (success, error, empty)
- `error_message` (string, optional): Error details if status is error

**Validation Rules**:
- `answer_text` must not be empty if status is success
- `source_chunks` array must contain valid SourceChunk objects
- `status` must be one of the defined values

### Source Attribution
**Description**: Information that identifies where in the book content the answer originated, including URL, page title, and section details
**Fields**:
- `chunk_id` (string): Unique identifier for the content chunk
- `text` (string): The actual content text from the source
- `url` (string): URL of the source page
- `page_title` (string): Title of the source page
- `section` (string): Section name in the source document
- `chunk_index` (integer): Index position of this chunk in the original content
- `source_url` (string): URL of the original source
- `chunk_hash` (string): Hash of the content for deduplication
- `score` (float): Relevance score from RAG system

**Validation Rules**:
- `text` must not be empty
- `url` must be a valid URL format
- `chunk_index` must be non-negative
- `score` must be between 0 and 1

### Chatbot State
**Description**: The current state of the chatbot UI to manage different user experiences
**Fields**:
- `state_type` (string): Current state (idle, loading, error, success, empty)
- `current_query` (UserQuery, optional): The query currently being processed
- `current_response` (ChatbotResponse, optional): The response currently displayed
- `error_details` (object, optional): Details when in error state
  - `message` (string): User-friendly error message
  - `code` (string): Technical error code
  - `timestamp` (datetime): When error occurred

**Validation Rules**:
- `state_type` must be one of the defined values
- `current_query` and `current_response` should match the current state

### Selected Text Context
**Description**: The portion of text on the current page that the user has selected to scope their question to
**Fields**:
- `text` (string): The selected text content
- `start_position` (integer): Starting character position in the page content
- `end_position` (integer): Ending character position in the page content
- `element_id` (string, optional): ID of the DOM element containing the selection
- `page_url` (string): URL of the page where text was selected
- `timestamp` (datetime): When the text was selected

**Validation Rules**:
- `text` must not be empty
- `start_position` must be less than `end_position`
- `start_position` and `end_position` must be non-negative

## Relationships

### User Query → Source Attribution
- One query can result in multiple source attributions (one-to-many)
- Relationship: A single query may return multiple relevant content chunks with source attribution

### Chatbot Response → Source Attribution
- One response contains multiple source attributions (one-to-many)
- Relationship: Each chatbot response aggregates multiple source attributions

### Chatbot State → User Query
- One state can reference one user query (one-to-one, optional)
- Relationship: Current state may be associated with a specific query

## State Transitions

### Chatbot States
1. **Idle**: Initial state, waiting for user input
2. **Loading**: Query submitted, waiting for API response
3. **Success**: Response received and displayed
4. **Error**: Error occurred during processing
5. **Empty**: No relevant content found for query

### State Transitions
- Idle → Loading: When user submits a query
- Loading → Success: When API returns successful response
- Loading → Error: When API returns error or timeout occurs
- Loading → Empty: When API returns no relevant content
- Error → Idle: When user dismisses error or retries
- Success → Idle: When user submits new query

## API Contract

### Query Request
```
POST /query
Content-Type: application/json

Request Body:
{
  "query": "string (required) - The user question",
  "selected_text": "string (optional) - Text selected on the current page",
  "context": {
    "page_url": "string - Current page URL",
    "page_title": "string - Current page title",
    "section": "string - Current section name"
  },
  "top_k": "integer (optional) - Number of results to return, default 5"
}
```

### Query Response
```
{
  "response_id": "string - Unique identifier for the response",
  "query_id": "string - Reference to the original query",
  "answer_text": "string - The main answer text",
  "source_chunks": [
    {
      "chunk_id": "string - Chunk identifier",
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
  "status": "string - success, error, or empty",
  "timestamp": "datetime - When response was generated"
}
```

### Error Response
```
{
  "response_id": "string - Unique identifier for the response",
  "query_id": "string - Reference to the original query (if available)",
  "status": "string - error",
  "error_message": "string - User-friendly error message",
  "error_code": "string - Technical error code",
  "timestamp": "datetime - When error occurred"
}
```