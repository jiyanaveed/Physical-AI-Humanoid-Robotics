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
from ..services.mock_agent_service import agent_service
from ..services.mock_qdrant_service import qdrant_service
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

@router.post("/")
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

        # Prepare results in the format expected by the frontend
        results = []
        for chunk in content_chunks:
            result = {
                "text": chunk.text,
                "url": chunk.url,
                "page_title": chunk.page_title,
                "section": chunk.section,
                "score": chunk.score,
                "source_url": chunk.source_url
            }
            results.append(result)

        # Generate answer text based on the content chunks
        if results:
            answer_text = f"Based on the documentation, here's information related to your query '{request.query}':\n\n"
            answer_text += "\n\n".join([chunk["text"] for chunk in results[:2]])  # Use first 2 chunks
        else:
            answer_text = f"I found no specific information about '{request.query}' in the documentation, but I can help you navigate to relevant sections of the book."

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

        # Return response in the format expected by the frontend
        return {
            "response_id": query_id,
            "query_id": query_id,
            "answer_text": answer_text,
            "source_chunks": results,
            "status": "success",
            "timestamp": datetime.utcnow().isoformat()
        }

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

        # Return error response in the format expected by the frontend
        return {
            "response_id": query_id,
            "query_id": query_id,
            "answer_text": f"An error occurred while processing your request: {error_msg}",
            "source_chunks": [],
            "status": "error",
            "error_message": error_msg,
            "timestamp": datetime.utcnow().isoformat()
        }

@router.post("/test")
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

        # Create mock content chunks for testing in the format expected by the frontend
        mock_chunks = [{
            "text": f"This is a mock response for your query: '{request.query}'. The Physical AI and Humanoid Robotics book covers topics like ROS 2 fundamentals, AI algorithms, control systems, and dynamics.",
            "url": "https://example.com/mock",
            "page_title": "Physical AI and Humanoid Robotics",
            "section": "Mock Section",
            "score": 0.95,
            "source_url": "https://example.com/mock"
        }]

        answer_text = f"Based on the documentation, here's information related to your query '{request.query}':\n\n"
        answer_text += mock_chunks[0]["text"]

        processing_time = (time.time() - start_time) * 1000

        # Log successful query processing
        logging_service.log_query_processing(
            query_text=request.query,
            response_summary="1 mock content chunk returned",
            processing_time=processing_time,
            status="success"
        )

        logger.info(f"Test query {query_id} processed successfully in {processing_time:.2f}ms")

        return {
            "response_id": query_id,
            "query_id": query_id,
            "answer_text": answer_text,
            "source_chunks": mock_chunks,
            "status": "success",
            "timestamp": datetime.utcnow().isoformat()
        }

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

        return {
            "response_id": query_id,
            "query_id": query_id,
            "answer_text": f"An error occurred while processing your request: {error_msg}",
            "source_chunks": [],
            "status": "error",
            "error_message": error_msg,
            "timestamp": datetime.utcnow().isoformat()
        }