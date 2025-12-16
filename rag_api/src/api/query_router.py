"""
Query router for the RAG Agent API.
"""
from fastapi import APIRouter, HTTPException
from datetime import datetime
import logging
import time
from typing import Dict, Any
from ..models.query import QueryRequest, QueryResponse
from ..models.response import ErrorResponse
from ..services.agent_service import agent_service
from ..services.qdrant_service import qdrant_service
from ..services.logging_service import logging_service
from ..config import config

logger = logging.getLogger(__name__)

router = APIRouter()

@router.get("/")
async def query_info():
    """
    Information about the query endpoint.
    """
    return {
        "message": "Query endpoint for RAG Agent",
        "usage": "POST to /query with a question in the request body",
        "timestamp": datetime.utcnow().isoformat()
    }

@router.post("/", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Accept user questions and return relevant content chunks.

    Args:
        request (QueryRequest): The query request containing the question

    Returns:
        QueryResponse: Response with relevant content chunks and metadata
    """
    start_time = time.time()
    query_id = f"query_{int(datetime.utcnow().timestamp())}"

    try:
        logger.info(f"Processing query {query_id}: {request.query[:50]}...")

        # Validate the request
        if not request.query.strip():
            error_msg = "Query cannot be empty"
            processing_time = (time.time() - start_time) * 1000
            logging_service.log_error(request.query, error_msg, processing_time)
            raise HTTPException(status_code=400, detail=error_msg)

        # Retrieve relevant content from Qdrant
        content_chunks = qdrant_service.get_content_chunks(
            query_text=request.query,
            top_k=request.top_k
        )

        # Process with the agent (for now, we'll return the content chunks directly)
        # In a real implementation, the agent would synthesize the information
        results = [chunk.model_dump() for chunk in content_chunks]

        # Calculate processing time in milliseconds
        processing_time = (time.time() - start_time) * 1000

        # Log successful query processing
        logging_service.log_query_processing(
            query_text=request.query,
            response_summary=f"{len(results)} content chunks returned",
            processing_time=processing_time,
            status="success"
        )

        logger.info(f"Query {query_id} processed successfully in {processing_time:.2f}ms")

        return QueryResponse(
            query_id=query_id,
            results=results,
            total_results=len(results),
            processing_time=processing_time,
            status="success"
        )

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        # Handle any other exceptions
        processing_time = (time.time() - start_time) * 1000
        error_msg = str(e)

        logger.error(f"Error processing query {query_id}: {error_msg}")

        # Log the error
        logging_service.log_error(
            query_text=request.query,
            error_message=error_msg,
            processing_time=processing_time
        )

        # Return error response
        return QueryResponse(
            query_id=query_id,
            results=[],
            total_results=0,
            processing_time=processing_time,
            status="error",
            error_message=error_msg
        )

@router.post("/test", response_model=QueryResponse)
async def test_query_endpoint(request: QueryRequest):
    """
    Test endpoint that simulates query processing without actually calling external services.
    Useful for testing the API structure.
    """
    start_time = time.time()
    query_id = f"test_query_{int(datetime.utcnow().timestamp())}"

    try:
        logger.info(f"Processing test query {query_id}: {request.query[:50]}...")

        # Simulate some processing time
        time.sleep(0.1)

        # Create mock content chunks for testing
        mock_chunks = [{
            "id": "mock_chunk_1",
            "text": f"Mock response for query: {request.query}",
            "url": "https://example.com/mock",
            "page_title": "Mock Page",
            "section": "Mock Section",
            "chunk_index": 0,
            "source_url": "https://example.com/mock",
            "chunk_hash": "mock_hash_123",
            "score": 0.95
        }]

        processing_time = (time.time() - start_time) * 1000

        # Log successful query processing
        logging_service.log_query_processing(
            query_text=request.query,
            response_summary="1 mock content chunk returned",
            processing_time=processing_time,
            status="success"
        )

        logger.info(f"Test query {query_id} processed successfully in {processing_time:.2f}ms")

        return QueryResponse(
            query_id=query_id,
            results=mock_chunks,
            total_results=1,
            processing_time=processing_time,
            status="success"
        )

    except Exception as e:
        processing_time = (time.time() - start_time) * 1000
        error_msg = str(e)

        logger.error(f"Error processing test query {query_id}: {error_msg}")

        # Log the error
        logging_service.log_error(
            query_text=request.query,
            error_message=error_msg,
            processing_time=processing_time
        )

        return QueryResponse(
            query_id=query_id,
            results=[],
            total_results=0,
            processing_time=processing_time,
            status="error",
            error_message=error_msg
        )