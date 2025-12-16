"""
Utility functions for the RAG Agent API.
"""
import logging
from typing import Optional
from fastapi import HTTPException
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def setup_logging():
    """Set up logging configuration for the application."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler()
        ]
    )

def log_info(message: str):
    """Log an info message."""
    logger.info(message)

def log_error(message: str):
    """Log an error message."""
    logger.error(message)

def log_warning(message: str):
    """Log a warning message."""
    logger.warning(message)

def validate_environment_vars(required_vars: list) -> bool:
    """Validate that required environment variables are set."""
    missing_vars = []
    for var in required_vars:
        if not var:
            missing_vars.append(var)

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        return False

    return True

def create_error_response(error_message: str, status_code: int = 500):
    """Create a standardized error response."""
    return {
        "status": "error",
        "error_message": error_message,
        "timestamp": datetime.utcnow().isoformat(),
        "status_code": status_code
    }