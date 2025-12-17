"""
Ingestion router for the RAG Agent API.
Provides endpoints for triggering the ingestion pipeline.
"""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from datetime import datetime
import logging
from typing import Dict, Any
import asyncio

from ..models.response import StandardResponse, APIResponse
from ..services.ingestion_service import ingestion_service
from ..services.logging_service import logging_service
from ..config import config

logger = logging.getLogger(__name__)

router = APIRouter()

class IngestionRequest:
    """Request model for ingestion endpoint"""
    def __init__(self, sitemap_url: str = None):
        self.sitemap_url = sitemap_url

# Note: We can't subclass Pydantic models this way in runtime, so we'll define a separate response model
from pydantic import BaseModel, Field
from typing import Optional

class IngestionResponse(BaseModel):
    """Response model for ingestion endpoint"""
    message: str = Field(..., description="Human-readable message about the operation")
    status: str = Field(..., description="Status of the request", example="success")
    data: Optional[Dict[str, Any]] = Field(None, description="Additional data related to the response")
    processing_time: Optional[float] = Field(None, description="Time taken to process in milliseconds")
    query_id: Optional[str] = Field(None, description="Unique identifier for the operation")

@router.get("/ingestion")
async def ingestion_info():
    """
    Information about the ingestion endpoint.
    """
    return {
        "message": "Ingestion endpoint for RAG Agent",
        "usage": "POST to /ingestion to trigger the content ingestion pipeline",
        "timestamp": datetime.utcnow().isoformat()
    }

@router.post("/ingestion", response_model=IngestionResponse)
async def ingestion_endpoint(background_tasks: BackgroundTasks, sitemap_url: str = None):
    """
    Trigger the ingestion pipeline to process content from a sitemap.

    Args:
        sitemap_url (str, optional): URL of the sitemap to process. If not provided, uses default from config

    Returns:
        IngestionResponse: Status of the ingestion process
    """
    start_time = datetime.utcnow()
    request_id = f"ingestion_{int(start_time.timestamp())}"

    try:
        logger.info(f"Starting ingestion process {request_id}")

        # Run the ingestion asynchronously to avoid blocking
        success = await asyncio.get_event_loop().run_in_executor(
            None, ingestion_service.ingest_book, sitemap_url
        )

        end_time = datetime.utcnow()
        processing_time = (end_time - start_time).total_seconds() * 1000  # Convert to milliseconds

        if success:
            message = f"Ingestion process {request_id} completed successfully"
            logger.info(message)

            # Log successful ingestion
            logging_service.log_ingestion_process(
                sitemap_url=sitemap_url or config.SITEMAP_URL,
                status="success",
                processing_time=processing_time
            )

            return IngestionResponse(
                message=message,
                status="success",
                data={"request_id": request_id, "processing_time_ms": processing_time},
                processing_time=processing_time,
                query_id=request_id
            )
        else:
            message = f"Ingestion process {request_id} completed with errors"
            logger.error(message)

            # Log failed ingestion
            logging_service.log_ingestion_process(
                sitemap_url=sitemap_url or config.SITEMAP_URL,
                status="failed",
                processing_time=processing_time
            )

            return IngestionResponse(
                message=message,
                status="error",
                data={"request_id": request_id, "processing_time_ms": processing_time},
                processing_time=processing_time,
                query_id=request_id
            )

    except Exception as e:
        processing_time = (datetime.utcnow() - start_time).total_seconds() * 1000

        error_msg = f"Error during ingestion process {request_id}: {str(e)}"
        logger.error(error_msg)

        # Log the error
        logging_service.log_error(
            query_text=f"Ingestion of {sitemap_url or 'default'}",
            error_message=str(e),
            processing_time=processing_time
        )

        raise HTTPException(status_code=500, detail=error_msg)

@router.post("/ingestion/validate")
async def validate_ingestion():
    """
    Validate the ingestion pipeline by checking Qdrant vectors and metadata correctness.

    Returns:
        IngestionResponse: Validation results
    """
    start_time = datetime.utcnow()
    request_id = f"validation_{int(start_time.timestamp())}"

    try:
        logger.info(f"Starting validation process {request_id}")

        # Run the validation asynchronously
        success = await asyncio.get_event_loop().run_in_executor(
            None, ingestion_service.validate_pipeline
        )

        end_time = datetime.utcnow()
        processing_time = (end_time - start_time).total_seconds() * 1000  # Convert to milliseconds

        if success:
            message = f"Validation process {request_id} completed successfully"
            logger.info(message)

            return IngestionResponse(
                message=message,
                status="success",
                data={"request_id": request_id, "processing_time_ms": processing_time, "validation_passed": True},
                processing_time=processing_time,
                query_id=request_id
            )
        else:
            message = f"Validation process {request_id} failed"
            logger.error(message)

            return IngestionResponse(
                message=message,
                status="error",
                data={"request_id": request_id, "processing_time_ms": processing_time, "validation_passed": False},
                processing_time=processing_time,
                query_id=request_id
            )

    except Exception as e:
        processing_time = (datetime.utcnow() - start_time).total_seconds() * 1000

        error_msg = f"Error during validation process {request_id}: {str(e)}"
        logger.error(error_msg)

        # Log the error
        logging_service.log_error(
            query_text="Ingestion validation",
            error_message=str(e),
            processing_time=processing_time
        )

        raise HTTPException(status_code=500, detail=error_msg)