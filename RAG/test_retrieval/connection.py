"""
Connection utilities for Qdrant retrieval testing.

This module provides functions for connecting to Qdrant Cloud and
verifying the connection to the RAD_Embedding collection.
"""
import os
import logging
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class QdrantConnection:
    """
    A class to handle Qdrant connection and basic operations.
    """

    def __init__(self):
        """
        Initialize the Qdrant connection with environment variables.
        """
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "RAD_Embedding"

        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key
        )

    def connect(self) -> bool:
        """
        Test connection to Qdrant collection and verify it exists.

        Returns:
            bool: True if connection is successful, False otherwise
        """
        try:
            # Test connection by getting collection info
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Successfully connected to Qdrant collection '{self.collection_name}'")
            logger.info(f"Collection points count: {collection_info.points_count}")
            logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
            logger.info(f"Distance: {collection_info.config.params.vectors.distance}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant collection '{self.collection_name}': {e}")
            return False

    def collection_exists(self) -> bool:
        """
        Check if the collection exists.

        Returns:
            bool: True if collection exists, False otherwise
        """
        try:
            self.client.get_collection(self.collection_name)
            return True
        except:
            return False

    def get_collection_info(self) -> Optional[dict]:
        """
        Get detailed information about the collection.

        Returns:
            Optional[dict]: Collection information or None if error
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'points_count': collection_info.points_count,
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'vectors_count': collection_info.vectors_count if hasattr(collection_info, 'vectors_count') else None
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return None

    def get_client(self):
        """
        Get the Qdrant client instance.

        Returns:
            QdrantClient: The Qdrant client instance
        """
        return self.client


def get_qdrant_client() -> QdrantClient:
    """
    Get a configured Qdrant client instance.

    Returns:
        QdrantClient: Configured Qdrant client
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

    return QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key
    )


def test_connection() -> bool:
    """
    Test connection to Qdrant collection.

    Returns:
        bool: True if connection is successful, False otherwise
    """
    try:
        client = get_qdrant_client()
        collection_name = "RAD_Embedding"

        # Test connection by getting collection info
        collection_info = client.get_collection(collection_name)
        logger.info(f"Successfully connected to Qdrant collection '{collection_name}'")
        logger.info(f"Collection points count: {collection_info.points_count}")
        logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
        logger.info(f"Distance: {collection_info.config.params.vectors.distance}")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant collection: {e}")
        return False