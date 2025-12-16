"""
Metadata and content validation functions for Qdrant retrieval testing.

This module provides functions for validating that retrieved content chunks
have correct metadata (URL, page title, section, chunk index) and that
the content is semantically relevant to the query.
"""
import logging
from typing import List, Dict, Tuple, Optional
from qdrant_client.http import models

# Configure logging
logger = logging.getLogger(__name__)


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


def validate_chunk_content(payload: Dict) -> bool:
    """
    Validate that the chunk content is meaningful and not empty.

    Args:
        payload (Dict): The payload containing the chunk content

    Returns:
        bool: True if content is valid, False otherwise
    """
    text = payload.get('text', '')

    # Check if text is empty or too short
    if not text or len(text.strip()) < 10:
        logger.warning("Chunk content is too short or empty")
        return False

    # Check if text looks like it might be just HTML tags or formatting
    stripped_text = text.strip()
    if stripped_text.startswith('<') and stripped_text.endswith('>'):
        logger.warning("Chunk content appears to be HTML tags")
        return False

    return True


def validate_semantic_relevance(query: str, result_payload: Dict, score_threshold: float = 0.3) -> bool:
    """
    Validate that the retrieved content is semantically relevant to the query.

    Args:
        query (str): The original query text
        result_payload (Dict): The payload of the retrieved result
        score_threshold (float): Minimum score threshold for relevance (default: 0.3)

    Returns:
        bool: True if content is semantically relevant, False otherwise
    """
    # Note: In a full implementation, we might use additional NLP techniques to validate relevance
    # For now, we'll rely on the Qdrant similarity score
    # This function can be enhanced with more sophisticated semantic validation

    # Check if the result payload contains the text field
    text = result_payload.get('text', '')
    if not text:
        return False

    # Basic check: see if any words from the query appear in the text (case insensitive)
    query_words = set(query.lower().split())
    text_words = set(text.lower().split())

    # If at least one word from the query appears in the text, consider it relevant
    # This is a basic heuristic - in practice, we'd rely more on the similarity score
    has_common_words = len(query_words.intersection(text_words)) > 0

    return has_common_words


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
    invalid_content = []
    valid_metadata_chunks = 0

    for result in results:
        payload = result['payload']

        # Validate metadata
        is_metadata_valid, missing_fields = validate_metadata(payload)
        if is_metadata_valid:
            valid_metadata_chunks += 1
        else:
            missing_metadata.extend(missing_fields)

        # Validate content
        is_content_valid = validate_chunk_content(payload)
        if not is_content_valid:
            invalid_content.append(result['id'])

        # Overall validity (both metadata and content)
        if is_metadata_valid and is_content_valid:
            valid_chunks += 1
        else:
            invalid_chunks += 1

    validation_summary = {
        'total_chunks': total_chunks,
        'valid_chunks': valid_chunks,
        'invalid_chunks': invalid_chunks,
        'valid_metadata_chunks': valid_metadata_chunks,
        'invalid_content_count': len(invalid_content),
        'validity_percentage': (valid_chunks / total_chunks * 100) if total_chunks > 0 else 0,
        'metadata_validity_percentage': (valid_metadata_chunks / total_chunks * 100) if total_chunks > 0 else 0,
        'missing_metadata_fields': list(set(missing_metadata)),  # Unique missing fields
        'invalid_content_chunk_ids': invalid_content
    }

    logger.info(f"Validation summary: {valid_chunks}/{total_chunks} chunks fully valid ({validation_summary['validity_percentage']:.1f}%)")
    logger.info(f"Metadata validation: {valid_metadata_chunks}/{total_chunks} chunks with valid metadata ({validation_summary['metadata_validity_percentage']:.1f}%)")

    return validation_summary


def validate_relevance_of_results(query: str, results: List[Dict], score_threshold: float = 0.3) -> Dict:
    """
    Validate the semantic relevance of retrieved results to the query.

    Args:
        query (str): The original query text
        results (List[Dict]): List of retrieved points with scores
        score_threshold (float): Minimum score threshold for relevance

    Returns:
        Dict: Relevance validation summary
    """
    total_results = len(results)
    relevant_results = 0
    high_score_results = 0
    medium_score_results = 0
    low_score_results = 0

    for result in results:
        score = result['score']
        payload = result['payload']

        # Count results by score range
        if score >= score_threshold:
            high_score_results += 1
        elif score >= score_threshold / 2:
            medium_score_results += 1
        else:
            low_score_results += 1

        # Check semantic relevance
        if validate_semantic_relevance(query, payload, score_threshold):
            relevant_results += 1

    relevance_summary = {
        'total_results': total_results,
        'relevant_results': relevant_results,
        'high_score_results': high_score_results,  # Score >= threshold
        'medium_score_results': medium_score_results,  # Score >= threshold/2
        'low_score_results': low_score_results,  # Score < threshold/2
        'relevance_percentage': (relevant_results / total_results * 100) if total_results > 0 else 0,
        'high_score_percentage': (high_score_results / total_results * 100) if total_results > 0 else 0,
        'score_threshold': score_threshold
    }

    logger.info(f"Relevance validation: {relevant_results}/{total_results} results relevant ({relevance_summary['relevance_percentage']:.1f}%)")
    logger.info(f"High score results: {high_score_results}/{total_results} ({relevance_summary['high_score_percentage']:.1f}%)")

    return relevance_summary


def detect_duplicate_chunks(results: List[Dict]) -> Dict:
    """
    Detect duplicate content chunks in the results.

    Args:
        results (List[Dict]): List of retrieved points

    Returns:
        Dict: Duplicate detection summary
    """
    chunk_hashes = []
    duplicate_hashes = []
    unique_hashes = set()

    for result in results:
        chunk_hash = result['payload'].get('chunk_hash')
        if chunk_hash:
            if chunk_hash in unique_hashes:
                if chunk_hash not in duplicate_hashes:
                    duplicate_hashes.append(chunk_hash)
            else:
                unique_hashes.add(chunk_hash)
            chunk_hashes.append(chunk_hash)

    duplicate_summary = {
        'total_hashes': len(chunk_hashes),
        'unique_hashes': len(unique_hashes),
        'duplicate_count': len(duplicate_hashes),
        'duplicate_hashes': duplicate_hashes,
        'duplicates_detected': len(duplicate_hashes) > 0
    }

    if duplicate_summary['duplicates_detected']:
        logger.warning(f"Duplicate chunks detected: {len(duplicate_hashes)} unique duplicates out of {len(chunk_hashes)} total")
    else:
        logger.info("No duplicate chunks detected")

    return duplicate_summary


def validate_metadata_integrity(payload: Dict) -> Dict:
    """
    Perform comprehensive validation of metadata integrity.

    Args:
        payload (Dict): The payload to validate

    Returns:
        Dict: Detailed metadata validation results
    """
    validation_results = {
        'url_valid': False,
        'page_title_valid': False,
        'text_valid': False,
        'chunk_id_valid': False,
        'chunk_index_valid': False,
        'source_url_valid': False,
        'chunk_hash_valid': False,
        'all_valid': False
    }

    # Validate URL
    url = payload.get('url')
    if url and isinstance(url, str) and url.startswith(('http://', 'https://')):
        validation_results['url_valid'] = True

    # Validate page title
    page_title = payload.get('page_title')
    if page_title and isinstance(page_title, str) and len(page_title.strip()) > 0:
        validation_results['page_title_valid'] = True

    # Validate text
    text = payload.get('text')
    if text and isinstance(text, str) and len(text.strip()) > 0:
        validation_results['text_valid'] = True

    # Validate chunk_id
    chunk_id = payload.get('chunk_id')
    if chunk_id is not None:
        validation_results['chunk_id_valid'] = True

    # Validate chunk_index
    chunk_index = payload.get('chunk_index')
    if isinstance(chunk_index, int) and chunk_index >= 0:
        validation_results['chunk_index_valid'] = True

    # Validate source_url
    source_url = payload.get('source_url')
    if source_url and isinstance(source_url, str) and source_url.startswith(('http://', 'https://')):
        validation_results['source_url_valid'] = True

    # Validate chunk_hash
    chunk_hash = payload.get('chunk_hash')
    if chunk_hash and isinstance(chunk_hash, str) and len(chunk_hash) > 0:
        validation_results['chunk_hash_valid'] = True

    # Overall validity
    validation_results['all_valid'] = all(validation_results.values())

    return validation_results


def validate_all_retrieved_results(results: List[Dict], query: str = "") -> Dict:
    """
    Perform comprehensive validation of all retrieved results.

    Args:
        results (List[Dict]): List of retrieved points
        query (str): Optional query for relevance validation

    Returns:
        Dict: Comprehensive validation results
    """
    # Perform individual validations
    metadata_validation = validate_retrieved_chunks(results)
    duplicate_validation = detect_duplicate_chunks(results)

    relevance_validation = {}
    if query:
        relevance_validation = validate_relevance_of_results(query, results)

    # Combine all validation results
    comprehensive_validation = {
        'metadata_validation': metadata_validation,
        'duplicate_validation': duplicate_validation,
        'relevance_validation': relevance_validation,
        'overall_status': 'success' if metadata_validation['validity_percentage'] >= 90 and not duplicate_validation['duplicates_detected'] else 'warning'
    }

    logger.info(f"Comprehensive validation completed. Overall status: {comprehensive_validation['overall_status']}")
    return comprehensive_validation