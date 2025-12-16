"""
Query execution functions for Qdrant retrieval testing.

This module provides functions for executing vector similarity searches
against the Qdrant collection based on sample text or keywords.
"""
import os
import logging
from typing import List, Dict, Optional, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)

# Initialize Cohere client
cohere_api_key = os.getenv("COHERE_API_KEY")
if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")
cohere_client = cohere.Client(cohere_api_key)
EMBED_MODEL = "embed-english-v3.0"


def embed_query_text(query_text: str) -> List[float]:
    """
    Generate embedding for query text using Cohere.

    Args:
        query_text (str): The query text to embed

    Returns:
        List[float]: The embedding vector
    """
    try:
        response = cohere_client.embed(
            model=EMBED_MODEL,
            input_type="search_query",  # Use search_query for query text
            texts=[query_text],
        )
        return response.embeddings[0]  # Return the first embedding
    except Exception as e:
        logger.error(f"Error generating embedding for query '{query_text}': {e}")
        raise


def execute_retrieval_query(
    query_text: str,
    top_k: int = 5,
    collection_name: str = "RAD_Embedding"
) -> List[Dict]:
    """
    Execute a retrieval query against the Qdrant collection.

    Args:
        query_text (str): The query text to search for
        top_k (int): Number of results to return (default: 5)
        collection_name (str): Name of the collection to search in

    Returns:
        List[Dict]: List of retrieved points with payload and score
    """
    try:
        # Generate embedding for the query text
        query_vector = embed_query_text(query_text)

        # Get Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Execute search in Qdrant using the query_points API method
        from qdrant_client.http import models

        search_result = qdrant.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results - query_points returns a QueryResponse object with points attribute
        results = []
        for point in search_result.points:
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
        return []


def execute_keyword_search(
    keywords: List[str],
    top_k: int = 5,
    collection_name: str = "RAD_Embedding"
) -> List[Dict]:
    """
    Execute a keyword-based search against the Qdrant collection.

    Args:
        keywords (List[str]): List of keywords to search for
        top_k (int): Number of results to return (default: 5)
        collection_name (str): Name of the collection to search in

    Returns:
        List[Dict]: List of retrieved points with payload and score
    """
    try:
        # Create a text query that includes all keywords
        query_text = " ".join(keywords)

        # Generate embedding for the query text
        query_vector = embed_query_text(query_text)

        # Get Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Execute search in Qdrant using the query_points API method
        from qdrant_client.http import models

        search_result = qdrant.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Filter results to ensure they contain the keywords
        filtered_results = []
        keyword_lower = [kw.lower() for kw in keywords]

        for point in search_result.points:
            # Check if the text contains any of the keywords (case insensitive)
            text = point.payload.get('text', '').lower()
            contains_keyword = any(kw in text for kw in keyword_lower)

            if contains_keyword or len(keywords) == 0:  # If no keywords provided, return all
                result = {
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload
                }
                filtered_results.append(result)

        logger.info(f"Retrieved {len(filtered_results)} keyword-based results for keywords: {keywords}")
        return filtered_results

    except Exception as e:
        logger.error(f"Error executing keyword search for '{keywords}': {e}")
        return []


def execute_batch_queries(
    queries: List[str],
    top_k: int = 5,
    collection_name: str = "RAD_Embedding"
) -> Dict[str, List[Dict]]:
    """
    Execute multiple queries in batch.

    Args:
        queries (List[str]): List of query texts to execute
        top_k (int): Number of results to return for each query (default: 5)
        collection_name (str): Name of the collection to search in

    Returns:
        Dict[str, List[Dict]]: Dictionary mapping query text to results
    """
    results = {}
    for query in queries:
        query_results = execute_retrieval_query(query, top_k, collection_name)
        results[query] = query_results
    return results


def test_empty_query() -> List[Dict]:
    """
    Test handling of empty queries.

    Returns:
        List[Dict]: Results for empty query (should handle gracefully)
    """
    try:
        return execute_retrieval_query("")
    except Exception as e:
        logger.warning(f"Empty query test handled error: {e}")
        return []


def test_no_match_query() -> List[Dict]:
    """
    Test handling of queries that are unlikely to match any content.

    Returns:
        List[Dict]: Results for no-match query
    """
    try:
        # Use a query that's unlikely to match any content in the collection
        return execute_retrieval_query("xyz123_unlikely_to_match_query_abc789")
    except Exception as e:
        logger.warning(f"No-match query test handled error: {e}")
        return []


def get_similar_chunks_by_id(
    chunk_id: str,
    top_k: int = 5,
    collection_name: str = "RAD_Embedding"
) -> List[Dict]:
    """
    Find similar chunks to a specific chunk by its ID.

    Args:
        chunk_id (str): The ID of the chunk to find similar ones to
        top_k (int): Number of similar results to return (default: 5)
        collection_name (str): Name of the collection to search in

    Returns:
        List[Dict]: List of similar chunks with payload and score
    """
    try:
        # Get Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Get the vector for the specified chunk
        points = qdrant.retrieve(
            collection_name=collection_name,
            ids=[chunk_id],
            with_vectors=True
        )

        if not points:
            logger.warning(f"Chunk with ID {chunk_id} not found")
            return []

        # Use the retrieved vector to find similar chunks
        search_result = qdrant.search(
            collection_name=collection_name,
            query_vector=points[0].vector,
            limit=top_k + 1,  # Get one more to exclude the original chunk
            with_payload=True,
            with_vectors=False
        )

        # Format results, excluding the original chunk
        results = []
        for point in search_result:
            if point.id != chunk_id:  # Exclude the original chunk
                result = {
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload
                }
                results.append(result)
                if len(results) >= top_k:  # Stop when we have enough results
                    break

        logger.info(f"Found {len(results)} similar chunks for chunk ID {chunk_id}")
        return results

    except Exception as e:
        logger.error(f"Error finding similar chunks for ID {chunk_id}: {e}")
        return []