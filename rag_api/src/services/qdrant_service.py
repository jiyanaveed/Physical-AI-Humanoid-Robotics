"""
Qdrant service for the RAG Agent API that handles vector database operations.
"""
import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from ..config import config
from ..models.content_chunk import ContentChunk

# Configure logging
logger = logging.getLogger(__name__)

class QdrantService:
    """Service class to handle Qdrant vector database operations for retrieval."""

    def __init__(self):
        """Initialize the Qdrant service with client and Cohere client."""
        if not config.QDRANT_URL or not config.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        if not config.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )

        self.cohere_client = cohere.Client(config.COHERE_API_KEY)
        self.collection_name = config.COLLECTION_NAME

        logger.info(f"Qdrant service initialized with collection: {self.collection_name}")

    def embed_query_text(self, query_text: str) -> List[float]:
        """
        Generate embedding for query text using Cohere.

        Args:
            query_text (str): The query text to embed

        Returns:
            List[float]: The embedding vector
        """
        try:
            response = self.cohere_client.embed(
                model="embed-english-v3.0",
                input_type="search_query",  # Use search_query for query text
                texts=[query_text],
            )
            return response.embeddings[0]  # Return the first embedding
        except Exception as e:
            logger.error(f"Error generating embedding for query '{query_text}': {e}")
            raise

    def execute_retrieval_query(
        self,
        query_text: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Execute a retrieval query against the Qdrant collection.

        Args:
            query_text (str): The query text to search for
            top_k (int): Number of results to return (default: 5)

        Returns:
            List[Dict]: List of retrieved points with payload and score
        """
        try:
            logger.info(f"Executing retrieval query: {query_text[:50]}...")

            # Generate embedding for the query text
            query_vector = self.embed_query_text(query_text)

            # Execute search in Qdrant
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for point in search_result:
                result = {
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} results for query: '{query_text[:50]}{'...' if len(query_text) > 50 else ''}'")
            return results

        except Exception as e:
            logger.error(f"Error executing retrieval query for '{query_text}': {e}")
            raise

    def get_content_chunks(
        self,
        query_text: str,
        top_k: int = 5
    ) -> List[ContentChunk]:
        """
        Get content chunks from Qdrant that match the query.

        Args:
            query_text (str): The query text to search for
            top_k (int): Number of results to return (default: 5)

        Returns:
            List[ContentChunk]: List of content chunks with metadata
        """
        try:
            results = self.execute_retrieval_query(query_text, top_k)

            content_chunks = []
            for result in results:
                payload = result['payload']

                # Create ContentChunk from payload
                chunk = ContentChunk(
                    id=result['id'],
                    text=payload.get('text', ''),
                    url=payload.get('url', ''),
                    page_title=payload.get('page_title', ''),
                    section=payload.get('section', ''),
                    chunk_index=payload.get('chunk_index', 0),
                    source_url=payload.get('source_url', ''),
                    chunk_hash=payload.get('chunk_hash', ''),
                    score=result['score']
                )
                content_chunks.append(chunk)

            logger.info(f"Formatted {len(content_chunks)} content chunks from Qdrant results")
            return content_chunks

        except Exception as e:
            logger.error(f"Error getting content chunks for query '{query_text}': {e}")
            raise

# Global qdrant service instance
qdrant_service = QdrantService()