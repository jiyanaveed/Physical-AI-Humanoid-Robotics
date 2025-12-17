"""
Data models for API Responses in the RAG Agent API.
"""
from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from .content_chunk import ContentChunk

class APIResponse(BaseModel):
    """The structured output returned to the client, containing relevant content chunks and metadata"""
    query_id: str = Field(..., description="Unique identifier for the query")
    results: List[ContentChunk] = Field(..., description="Array of relevant content chunks")
    total_results: int = Field(..., description="Total number of results returned")
    processing_time: float = Field(..., description="Time taken to process the query in milliseconds")
    status: str = Field(..., description="Status of the request", example="success")
    error_message: Optional[str] = Field(None, description="Error details if status is error")

    class Config:
        json_schema_extra = {
            "example": {
                "query_id": "query_12345",
                "results": [
                    {
                        "id": "chunk_67890",
                        "text": "Robotics is an interdisciplinary field that includes mechanical engineering, electrical engineering, and computer science...",
                        "url": "https://example.com/book/chapter1",
                        "page_title": "Introduction to Robotics",
                        "section": "Chapter 1.1",
                        "chunk_index": 1,
                        "source_url": "https://example.com/book",
                        "chunk_hash": "a1b2c3d4e5f6",
                        "score": 0.95
                    }
                ],
                "total_results": 1,
                "processing_time": 123.45,
                "status": "success"
            }
        }

class ErrorResponse(BaseModel):
    """Error response model"""
    query_id: Optional[str] = Field(None, description="Unique identifier for the query (if available)")
    status: str = Field(..., description="Status of the request", example="error")
    error_message: str = Field(..., description="Description of the error")
    processing_time: Optional[float] = Field(None, description="Time taken before error occurred in milliseconds")

    class Config:
        json_schema_extra = {
            "example": {
                "query_id": "query_12345",
                "status": "error",
                "error_message": "Invalid query format",
                "processing_time": 10.23
            }
        }

class StandardResponse(BaseModel):
    """Standard response model for various API operations"""
    message: str = Field(..., description="Human-readable message about the operation")
    status: str = Field(..., description="Status of the request", example="success")
    data: Optional[dict] = Field(None, description="Additional data related to the response")
    processing_time: Optional[float] = Field(None, description="Time taken to process in milliseconds")
    query_id: Optional[str] = Field(None, description="Unique identifier for the operation")

    class Config:
        json_schema_extra = {
            "example": {
                "message": "Operation completed successfully",
                "status": "success",
                "data": {"request_id": "req_12345", "processing_time_ms": 123.45},
                "processing_time": 123.45,
                "query_id": "req_12345"
            }
        }

class QueryLog(BaseModel):
    """A record of user queries, system responses, processing time, and any errors encountered"""
    log_id: str = Field(..., description="Unique identifier for the log entry")
    query_text: str = Field(..., description="The original query text")
    response_summary: str = Field(..., description="Brief summary of the response")
    processing_time: float = Field(..., description="Time taken to process in milliseconds", ge=0)
    timestamp: datetime = Field(..., description="When the query was processed")
    status: str = Field(..., description="Status of processing", example="success")
    error_details: Optional[str] = Field(None, description="Details if an error occurred")

    class Config:
        json_schema_extra = {
            "example": {
                "log_id": "log_111222",
                "query_text": "What are the key concepts in robotics?",
                "response_summary": "3 content chunks returned",
                "processing_time": 123.45,
                "timestamp": "2025-12-17T10:30:00Z",
                "status": "success",
                "error_details": None
            }
        }