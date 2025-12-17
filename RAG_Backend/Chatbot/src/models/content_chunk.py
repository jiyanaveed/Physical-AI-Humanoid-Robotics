"""
Data models for Content Chunk in the RAG Agent API.
"""
from pydantic import BaseModel, Field
from typing import Optional

class ContentChunk(BaseModel):
    """A segment of book content retrieved from Qdrant that matches the user's query, including the text content and associated metadata"""
    id: str = Field(..., description="Unique identifier for the chunk")
    text: str = Field(..., description="The actual content text", min_length=1)
    url: str = Field(..., description="URL of the source page")
    page_title: str = Field(..., description="Title of the source page")
    section: str = Field(..., description="Section name in the source document")
    chunk_index: int = Field(..., description="Index position of this chunk in the original content", ge=0)
    source_url: str = Field(..., description="URL of the original source")
    chunk_hash: str = Field(..., description="Hash of the content for deduplication")
    score: float = Field(..., description="Relevance score from Qdrant search")

    class Config:
        json_schema_extra = {
            "example": {
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
        }