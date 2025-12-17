"""
Data models for User Query in the RAG Agent API.
"""
from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

class UserQuery(BaseModel):
    """A text-based question submitted by the user that requires relevant book content as a response"""
    query_text: str = Field(..., description="The actual question text from the user", min_length=1, max_length=1000)
    query_id: Optional[str] = Field(None, description="Unique identifier for the query")
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow, description="When the query was submitted")
    user_id: Optional[str] = Field(None, description="Identifier for the user (if implemented later)")

    class Config:
        json_schema_extra = {
            "example": {
                "query_text": "What are the key concepts in robotics?",
                "query_id": "query_12345",
                "timestamp": "2025-12-17T10:30:00Z",
                "user_id": "user_67890"
            }
        }

class QueryRequest(BaseModel):
    """Request model for the query endpoint"""
    query: str = Field(..., description="The user's question", min_length=1, max_length=1000)
    top_k: Optional[int] = Field(default=5, ge=1, le=20, description="Number of results to return (default: 5)")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the key concepts in robotics?",
                "top_k": 5
            }
        }

class QueryResponse(BaseModel):
    """Response model for the query endpoint"""
    query_id: str = Field(..., description="Unique identifier for the query")
    results: list = Field(..., description="Array of relevant content chunks")
    total_results: int = Field(..., description="Total number of results returned")
    processing_time: float = Field(..., description="Time taken to process the query in milliseconds")
    status: str = Field(..., description="Status of the request", example="success")
    error_message: Optional[str] = Field(None, description="Error details if status is error")