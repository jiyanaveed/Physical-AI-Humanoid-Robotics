"""
Logging service for the RAG Agent API that handles query processing logs.
"""
import logging
from datetime import datetime
from typing import Dict, Any, Optional
from ..models.response import QueryLog

class LoggingService:
    """Service class to handle logging for query processing and results."""

    def __init__(self):
        """Initialize the logging service."""
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

        # Create handler if not already configured
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def log_query_processing(
        self,
        query_text: str,
        response_summary: str,
        processing_time: float,
        status: str = "success",
        error_details: Optional[str] = None
    ) -> QueryLog:
        """
        Log query processing details.

        Args:
            query_text (str): The original query text
            response_summary (str): Brief summary of the response
            processing_time (float): Time taken to process in milliseconds
            status (str): Status of processing (default: "success")
            error_details (str, optional): Details if an error occurred

        Returns:
            QueryLog: The created log entry
        """
        log_entry = QueryLog(
            log_id=f"log_{int(datetime.utcnow().timestamp())}",
            query_text=query_text,
            response_summary=response_summary,
            processing_time=processing_time,
            timestamp=datetime.utcnow(),
            status=status,
            error_details=error_details
        )

        # Log to configured logger
        if status == "success":
            self.logger.info(
                f"Query processed - ID: {log_entry.log_id}, "
                f"Query: '{query_text[:50]}...', "
                f"Time: {processing_time}ms, "
                f"Status: {status}"
            )
        else:
            self.logger.error(
                f"Query failed - ID: {log_entry.log_id}, "
                f"Query: '{query_text[:50]}...', "
                f"Time: {processing_time}ms, "
                f"Status: {status}, "
                f"Error: {error_details}"
            )

        return log_entry

    def log_error(
        self,
        query_text: str,
        error_message: str,
        processing_time: float
    ) -> QueryLog:
        """
        Log an error during query processing.

        Args:
            query_text (str): The original query text
            error_message (str): The error message
            processing_time (float): Time taken before error in milliseconds

        Returns:
            QueryLog: The created log entry
        """
        return self.log_query_processing(
            query_text=query_text,
            response_summary="Error occurred during processing",
            processing_time=processing_time,
            status="error",
            error_details=error_message
        )

# Global logging service instance
logging_service = LoggingService()