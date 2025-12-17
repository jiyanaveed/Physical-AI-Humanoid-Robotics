"""
Health check router for the RAG Agent API.
"""
from fastapi import APIRouter
from datetime import datetime

router = APIRouter()

@router.get("/")
async def health_check():
    """
    Health check endpoint to verify the API is operational.
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "message": "RAG Agent API is running"
    }