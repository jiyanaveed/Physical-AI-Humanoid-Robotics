"""
Data Retrieval Pipeline Testing System

This script implements a testing system that connects to the Qdrant collection,
executes sample retrieval queries, validates returned content chunks and metadata,
handles edge cases, and logs retrieval results for correctness verification.
"""
import os
import time
import logging
from typing import List, Dict, Optional, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('retrieval_test.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Initialize Cohere client
cohere_api_key = os.getenv("COHERE_API_KEY")
if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")
cohere_client = cohere.Client(cohere_api_key)
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_url or not qdrant_api_key:
    raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

COLLECTION_NAME = "RAD_Embedding"  # As specified in requirements


def connect_to_qdrant() -> bool:
    """
    Test connection to Qdrant collection and verify it exists.

    Returns:
        bool: True if connection is successful, False otherwise
    """
    try:
        # Test connection by getting collection info
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        logger.info(f"Successfully connected to Qdrant collection '{COLLECTION_NAME}'")
        logger.info(f"Collection points count: {collection_info.points_count}")
        logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
        logger.info(f"Distance: {collection_info.config.params.vectors.distance}")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant collection '{COLLECTION_NAME}': {e}")
        return False


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


def execute_retrieval_query(query_text: str, top_k: int = 5) -> List[Dict]:
    """
    Execute a retrieval query against the Qdrant collection.

    Args:
        query_text (str): The query text to search for
        top_k (int): Number of results to return (default: 5)

    Returns:
        List[Dict]: List of retrieved points with payload and score
    """
    try:
        # Generate embedding for the query text
        query_vector = embed_query_text(query_text)

        # Execute search in Qdrant using the query_points API method
        from qdrant_client.http import models

        search_result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
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


def validate_metadata(payload: Dict) -> Tuple[bool, List[str]]:
    """
    Validate that all required metadata fields are present and correct.

    Args:
        payload (Dict): The payload to validate

    Returns:
        Tuple[bool, List[str]]: (is_valid, list_of_missing_fields)
    """
    required_fields = ["url", "page_title", "text", "chunk_id", "chunk_index", "source_url", "chunk_hash"]
    missing_fields = []

    for field in required_fields:
        if field not in payload:
            missing_fields.append(field)
        elif payload[field] is None:
            missing_fields.append(field)
        elif isinstance(payload[field], str) and payload[field].strip() == "":
            missing_fields.append(field)

    is_valid = len(missing_fields) == 0
    if not is_valid:
        logger.warning(f"Missing metadata fields: {missing_fields}")

    return is_valid, missing_fields


def validate_retrieved_chunks(results: List[Dict]) -> Dict:
    """
    Validate all retrieved chunks and their metadata.

    Args:
        results (List[Dict]): List of retrieved points

    Returns:
        Dict: Validation summary with counts and issues
    """
    total_chunks = len(results)
    valid_chunks = 0
    invalid_chunks = 0
    missing_metadata = []

    for result in results:
        is_valid, missing_fields = validate_metadata(result['payload'])
        if is_valid:
            valid_chunks += 1
        else:
            invalid_chunks += 1
            missing_metadata.extend(missing_fields)

    validation_summary = {
        'total_chunks': total_chunks,
        'valid_chunks': valid_chunks,
        'invalid_chunks': invalid_chunks,
        'validity_percentage': (valid_chunks / total_chunks * 100) if total_chunks > 0 else 0,
        'missing_metadata_fields': list(set(missing_metadata))  # Unique missing fields
    }

    logger.info(f"Validation summary: {valid_chunks}/{total_chunks} chunks valid ({validation_summary['validity_percentage']:.1f}%)")

    return validation_summary


def test_edge_cases() -> Dict:
    """
    Test edge cases: empty queries, missing results, duplicates.

    Returns:
        Dict: Results of edge case testing
    """
    logger.info("Testing edge cases...")

    edge_case_results = {
        'empty_query_test': None,
        'no_results_test': None,
        'duplicate_handling_test': None
    }

    # Test 1: Empty query
    try:
        empty_results = execute_retrieval_query("")
        edge_case_results['empty_query_test'] = {
            'status': 'success' if isinstance(empty_results, list) else 'error',
            'result_count': len(empty_results) if isinstance(empty_results, list) else 0
        }
    except Exception as e:
        logger.error(f"Empty query test failed: {e}")
        edge_case_results['empty_query_test'] = {
            'status': 'error',
            'error': str(e)
        }

    # Test 2: No results (using a query that's unlikely to match)
    try:
        no_match_results = execute_retrieval_query("xyz123_unlikely_to_match_query_abc789")
        edge_case_results['no_results_test'] = {
            'status': 'success',
            'result_count': len(no_match_results)
        }
    except Exception as e:
        logger.error(f"No results test failed: {e}")
        edge_case_results['no_results_test'] = {
            'status': 'error',
            'error': str(e)
        }

    # Test 3: Duplicate handling (check if we have any duplicate chunk hashes)
    try:
        # Get a sample of points to check for duplicates
        sample_results = execute_retrieval_query("test", top_k=10)
        chunk_hashes = [point['payload'].get('chunk_hash') for point in sample_results if point['payload'].get('chunk_hash')]
        unique_hashes = set(chunk_hashes)

        edge_case_results['duplicate_handling_test'] = {
            'status': 'success',
            'total_hashes': len(chunk_hashes),
            'unique_hashes': len(unique_hashes),
            'duplicates_found': len(chunk_hashes) != len(unique_hashes)
        }
    except Exception as e:
        logger.error(f"Duplicate handling test failed: {e}")
        edge_case_results['duplicate_handling_test'] = {
            'status': 'error',
            'error': str(e)
        }

    logger.info("Edge case testing completed")
    return edge_case_results


def log_retrieval_results(query: str, results: List[Dict], validation_summary: Dict) -> None:
    """
    Log retrieval results with detailed information.

    Args:
        query (str): The query that was executed
        results (List[Dict]): The retrieved results
        validation_summary (Dict): Validation summary
    """
    logger.info(f"--- Retrieval Results for Query: '{query}' ---")
    logger.info(f"Query length: {len(query)} characters")
    logger.info(f"Results count: {len(results)}")

    if results:
        logger.info("Top 3 results:")
        for i, result in enumerate(results[:3]):
            score = result['score']
            payload = result['payload']
            title = payload.get('page_title', 'No title')[:50]
            url = payload.get('url', 'No URL')[:50]
            logger.info(f"  {i+1}. Score: {score:.4f}, Title: '{title}...', URL: '{url}...'")

    logger.info(f"Validation: {validation_summary['valid_chunks']}/{validation_summary['total_chunks']} valid chunks")
    logger.info(f"Validity percentage: {validation_summary['validity_percentage']:.1f}%")

    if validation_summary['missing_metadata_fields']:
        logger.warning(f"Missing metadata fields: {validation_summary['missing_metadata_fields']}")


def run_sample_queries() -> List[Dict]:
    """
    Run a set of sample queries to test the retrieval pipeline.

    Returns:
        List[Dict]: Results for each query
    """
    sample_queries = [
        "AI humanoid robotics",
        "machine learning",
        "neural networks",
        "computer vision",
        "reinforcement learning",
        "robotics control"
    ]

    all_results = []

    for query in sample_queries:
        logger.info(f"Executing sample query: '{query}'")

        # Execute the query
        results = execute_retrieval_query(query, top_k=5)

        # Validate the results
        validation_summary = validate_retrieved_chunks(results)

        # Log the results
        log_retrieval_results(query, results, validation_summary)

        # Store the results
        query_result = {
            'query': query,
            'results': results,
            'validation_summary': validation_summary,
            'timestamp': time.time()
        }
        all_results.append(query_result)

    return all_results


def generate_test_summary(all_results: List[Dict], edge_case_results: Dict) -> Dict:
    """
    Generate a summary report of the testing outcomes.

    Args:
        all_results (List[Dict]): Results from all sample queries
        edge_case_results (Dict): Results from edge case testing

    Returns:
        Dict: Summary report
    """
    total_queries = len(all_results)
    total_chunks = sum(result['validation_summary']['total_chunks'] for result in all_results)
    total_valid_chunks = sum(result['validation_summary']['valid_chunks'] for result in all_results)
    avg_validity_percentage = sum(result['validation_summary']['validity_percentage'] for result in all_results) / total_queries if total_queries > 0 else 0

    summary = {
        'total_queries_executed': total_queries,
        'total_chunks_retrieved': total_chunks,
        'total_valid_chunks': total_valid_chunks,
        'average_validity_percentage': avg_validity_percentage,
        'edge_case_results': edge_case_results,
        'overall_status': 'success' if avg_validity_percentage >= 70.0 and total_valid_chunks > 0 else 'warning'
    }

    logger.info("=== RETRIEVAL TESTING SUMMARY ===")
    logger.info(f"Total queries executed: {total_queries}")
    logger.info(f"Total chunks retrieved: {total_chunks}")
    logger.info(f"Total valid chunks: {total_valid_chunks}")
    logger.info(f"Average validity percentage: {avg_validity_percentage:.1f}%")
    logger.info(f"Overall status: {summary['overall_status']}")
    logger.info("Edge case results:")
    for test_name, result in edge_case_results.items():
        logger.info(f"  {test_name}: {result.get('status', 'unknown')}")

    return summary


def main():
    """
    Main function to run the retrieval pipeline testing.
    """
    logger.info("Starting Data Retrieval Pipeline Testing...")

    # Test connection to Qdrant
    if not connect_to_qdrant():
        logger.error("Failed to connect to Qdrant. Exiting.")
        return False

    # Run sample queries
    logger.info("Running sample retrieval queries...")
    all_results = run_sample_queries()

    # Test edge cases
    logger.info("Testing edge cases...")
    edge_case_results = test_edge_cases()

    # Generate summary report
    logger.info("Generating summary report...")
    summary = generate_test_summary(all_results, edge_case_results)

    logger.info("Data Retrieval Pipeline Testing completed!")

    return summary


if __name__ == "__main__":
    success = main()
    if success:
        print("\n🎉 Retrieval pipeline testing completed successfully!")
    else:
        print("\n❌ Retrieval pipeline testing failed!")