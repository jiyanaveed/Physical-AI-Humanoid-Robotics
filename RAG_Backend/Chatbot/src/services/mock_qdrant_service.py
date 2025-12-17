"""
Mock Qdrant service for the RAG Agent API that handles vector database operations without external dependencies.
"""
import os
import logging
from typing import List, Dict, Any, Optional
from ..config import config
from ..models.content_chunk import ContentChunk

# Configure logging
logger = logging.getLogger(__name__)

class MockQdrantService:
    """Mock service class to handle Qdrant operations without external dependencies."""

    def __init__(self):
        """Initialize the mock service."""
        logger.info("Mock Qdrant service initialized")

    def embed_query_text(self, query_text: str) -> List[float]:
        """
        Mock embedding function that returns a simple vector based on the text.
        """
        # Create a simple mock embedding based on the text
        # In a real implementation, this would call Cohere
        return [float(ord(c) % 100) for c in query_text[:50]] + [0.0] * (50 - len(query_text[:50]))

    def execute_retrieval_query(
        self,
        query_text: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Execute a mock retrieval query.
        """
        try:
            logger.info(f"Executing mock retrieval query: {query_text[:50]}...")

            # Create mock results based on the query
            mock_results = []
            for i in range(min(top_k, 3)):  # Return up to 3 mock results
                mock_result = {
                    'id': f'mock_chunk_{i+1}',
                    'score': 0.9 - (i * 0.1),  # Decreasing scores
                    'payload': {
                        'text': f"This is a mock response related to your query: '{query_text}'. Based on the Physical AI and Humanoid Robotics documentation, this topic covers important concepts in robotics and AI.",
                        'url': f'https://example.com/mock-doc-{i+1}',
                        'page_title': 'Mock Documentation Page',
                        'section': 'Mock Section',
                        'chunk_index': i,
                        'source_url': 'https://example.com/mock-source',
                        'chunk_hash': f'mock_hash_{i+1}'
                    }
                }
                mock_results.append(mock_result)

            logger.info(f"Mock retrieved {len(mock_results)} results for query: '{query_text[:50]}{'...' if len(query_text) > 50 else ''}'")
            return mock_results

        except Exception as e:
            logger.error(f"Error executing mock retrieval query for '{query_text}': {e}")
            raise

    def get_content_chunks(
        self,
        query_text: str,
        top_k: int = 5
    ) -> List[ContentChunk]:
        """
        Get mock content chunks that match the query.
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

            logger.info(f"Formatted {len(content_chunks)} mock content chunks")
            return content_chunks

        except Exception as e:
            logger.error(f"Error getting mock content chunks for query '{query_text}': {e}")
            raise

# Global mock qdrant service instance
def _get_qdrant_service():
    """Lazy initialization of qdrant service to handle missing dependencies."""
    try:
        # Try to initialize the real service
        from .qdrant_service import QdrantService
        return QdrantService()
    except Exception as e:
        logger.warning(f"Real Qdrant service failed to initialize: {e}")
        # Return mock service that works without external dependencies
        return MockQdrantService()

qdrant_service = _get_qdrant_service()