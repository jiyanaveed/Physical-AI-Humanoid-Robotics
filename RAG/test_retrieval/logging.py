"""
Logging and reporting utilities for Qdrant retrieval testing.

This module provides functions for logging retrieval operations,
generating performance metrics, and creating summary reports.
"""
import os
import time
import logging
from datetime import datetime
from typing import List, Dict, Any
from dataclasses import dataclass
from pathlib import Path

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


@dataclass
class RetrievalResult:
    """
    Data class to store retrieval result information.
    """
    query: str
    results: List[Dict]
    validation_summary: Dict
    relevance_summary: Dict
    execution_time: float
    timestamp: float


class RetrievalLogger:
    """
    A class to handle logging and reporting of retrieval operations.
    """

    def __init__(self, log_file: str = "retrieval_test.log"):
        """
        Initialize the retrieval logger.

        Args:
            log_file (str): Path to the log file
        """
        self.log_file = log_file
        self.results_log = []

        # Set up file handler for detailed logging
        self.file_handler = logging.FileHandler(log_file)
        self.file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        self.file_handler.setFormatter(formatter)

        # Add file handler to logger if not already present
        if not any(isinstance(handler, logging.FileHandler) for handler in logger.handlers):
            logger.addHandler(self.file_handler)

    def log_retrieval_operation(
        self,
        query: str,
        results: List[Dict],
        validation_summary: Dict,
        relevance_summary: Dict = None,
        execution_time: float = 0.0
    ) -> RetrievalResult:
        """
        Log a retrieval operation with detailed information.

        Args:
            query (str): The query that was executed
            results (List[Dict]): The retrieved results
            validation_summary (Dict): Validation summary
            relevance_summary (Dict): Relevance validation summary
            execution_time (float): Time taken for the operation

        Returns:
            RetrievalResult: The logged result object
        """
        timestamp = time.time()

        result = RetrievalResult(
            query=query,
            results=results,
            validation_summary=validation_summary,
            relevance_summary=relevance_summary or {},
            execution_time=execution_time,
            timestamp=timestamp
        )

        self.results_log.append(result)

        # Log detailed information
        logger.info(f"--- Retrieval Operation Log ---")
        logger.info(f"Query: '{query}'")
        logger.info(f"Execution time: {execution_time:.4f}s")
        logger.info(f"Results count: {len(results)}")
        logger.info(f"Validation: {validation_summary['valid_chunks']}/{validation_summary['total_chunks']} valid chunks")
        logger.info(f"Validity percentage: {validation_summary['validity_percentage']:.1f}%")

        if relevance_summary:
            logger.info(f"Relevance: {relevance_summary['relevant_results']}/{relevance_summary['total_results']} relevant results")
            logger.info(f"Relevance percentage: {relevance_summary['relevance_percentage']:.1f}%")

        if results:
            logger.info("Top 3 results:")
            for i, result in enumerate(results[:3]):
                score = result['score']
                payload = result['payload']
                title = payload.get('page_title', 'No title')[:50]
                url = payload.get('url', 'No URL')[:50]
                logger.info(f"  {i+1}. Score: {score:.4f}, Title: '{title}...', URL: '{url}...'")

        if validation_summary.get('missing_metadata_fields'):
            logger.warning(f"Missing metadata fields: {validation_summary['missing_metadata_fields']}")

        if validation_summary.get('invalid_content_chunk_ids'):
            logger.warning(f"Invalid content in chunks: {validation_summary['invalid_content_chunk_ids'][:5]}...")  # Show first 5

        return result

    def log_edge_case_test(self, test_name: str, test_result: Dict) -> None:
        """
        Log the results of an edge case test.

        Args:
            test_name (str): Name of the test
            test_result (Dict): Result of the test
        """
        logger.info(f"--- Edge Case Test: {test_name} ---")
        logger.info(f"Status: {test_result.get('status', 'unknown')}")
        if 'result_count' in test_result:
            logger.info(f"Result count: {test_result['result_count']}")
        if 'error' in test_result:
            logger.error(f"Error: {test_result['error']}")

    def generate_performance_report(self) -> Dict:
        """
        Generate a performance report based on logged operations.

        Returns:
            Dict: Performance metrics
        """
        if not self.results_log:
            logger.warning("No results to generate performance report from")
            return {}

        total_operations = len(self.results_log)
        total_results = sum(len(result.results) for result in self.results_log)
        avg_execution_time = sum(result.execution_time for result in self.results_log) / total_operations
        avg_validity_percentage = sum(result.validation_summary['validity_percentage'] for result in self.results_log) / total_operations

        performance_metrics = {
            'total_operations': total_operations,
            'total_results_retrieved': total_results,
            'average_results_per_operation': total_results / total_operations if total_operations > 0 else 0,
            'average_execution_time': avg_execution_time,
            'average_validity_percentage': avg_validity_percentage,
            'total_execution_time': sum(result.execution_time for result in self.results_log),
            'fastest_operation': min((result.execution_time for result in self.results_log), default=0),
            'slowest_operation': max((result.execution_time for result in self.results_log), default=0)
        }

        logger.info("=== PERFORMANCE REPORT ===")
        logger.info(f"Total operations: {performance_metrics['total_operations']}")
        logger.info(f"Total results retrieved: {performance_metrics['total_results_retrieved']}")
        logger.info(f"Average results per operation: {performance_metrics['average_results_per_operation']:.2f}")
        logger.info(f"Average execution time: {performance_metrics['average_execution_time']:.4f}s")
        logger.info(f"Average validity percentage: {performance_metrics['average_validity_percentage']:.1f}%")
        logger.info(f"Total execution time: {performance_metrics['total_execution_time']:.4f}s")
        logger.info(f"Fastest operation: {performance_metrics['fastest_operation']:.4f}s")
        logger.info(f"Slowest operation: {performance_metrics['slowest_operation']:.4f}s")

        return performance_metrics

    def generate_validation_report(self) -> Dict:
        """
        Generate a validation report based on logged operations.

        Returns:
            Dict: Validation metrics
        """
        if not self.results_log:
            logger.warning("No results to generate validation report from")
            return {}

        total_chunks = sum(result.validation_summary['total_chunks'] for result in self.results_log)
        total_valid_chunks = sum(result.validation_summary['valid_chunks'] for result in self.results_log)
        avg_validity_percentage = sum(result.validation_summary['validity_percentage'] for result in self.results_log) / len(self.results_log)

        # Collect all missing metadata fields
        all_missing_fields = []
        for result in self.results_log:
            all_missing_fields.extend(result.validation_summary.get('missing_metadata_fields', []))

        validation_metrics = {
            'total_chunks_processed': total_chunks,
            'total_valid_chunks': total_valid_chunks,
            'average_validity_percentage': avg_validity_percentage,
            'most_common_missing_fields': self._get_most_common_items(all_missing_fields, 5)
        }

        logger.info("=== VALIDATION REPORT ===")
        logger.info(f"Total chunks processed: {validation_metrics['total_chunks_processed']}")
        logger.info(f"Total valid chunks: {validation_metrics['total_valid_chunks']}")
        logger.info(f"Average validity percentage: {validation_metrics['average_validity_percentage']:.1f}%")
        if validation_metrics['most_common_missing_fields']:
            logger.info(f"Most common missing fields: {validation_metrics['most_common_missing_fields']}")

        return validation_metrics

    def generate_summary_report(self) -> Dict:
        """
        Generate a comprehensive summary report.

        Returns:
            Dict: Summary report
        """
        performance_report = self.generate_performance_report()
        validation_report = self.generate_validation_report()

        summary = {
            'timestamp': datetime.now().isoformat(),
            'performance': performance_report,
            'validation': validation_report,
            'total_operations': len(self.results_log),
            'status': 'success' if performance_report and validation_report.get('average_validity_percentage', 0) >= 70.0 else 'warning'
        }

        logger.info("=== SUMMARY REPORT ===")
        logger.info(f"Report generated at: {summary['timestamp']}")
        logger.info(f"Total operations: {summary['total_operations']}")
        logger.info(f"Overall status: {summary['status']}")

        return summary

    def _get_most_common_items(self, items_list: List[str], n: int) -> List[tuple]:
        """
        Get the most common items from a list.

        Args:
            items_list (List[str]): List of items
            n (int): Number of most common items to return

        Returns:
            List[tuple]: List of (item, count) tuples
        """
        if not items_list:
            return []

        item_counts = {}
        for item in items_list:
            item_counts[item] = item_counts.get(item, 0) + 1

        # Sort by count in descending order and return top n
        sorted_items = sorted(item_counts.items(), key=lambda x: x[1], reverse=True)
        return sorted_items[:n]

    def export_results_to_file(self, filename: str = "retrieval_results_export.json") -> None:
        """
        Export all logged results to a JSON file.

        Args:
            filename (str): Name of the export file
        """
        import json

        # Convert results to serializable format
        serializable_results = []
        for result in self.results_log:
            serializable_result = {
                'query': result.query,
                'results_count': len(result.results),
                'results': [
                    {
                        'id': r['id'],
                        'score': r['score'],
                        'payload_keys': list(r['payload'].keys())
                    } for r in result.results
                ],
                'validation_summary': result.validation_summary,
                'relevance_summary': result.relevance_summary,
                'execution_time': result.execution_time,
                'timestamp': datetime.fromtimestamp(result.timestamp).isoformat()
            }
            serializable_results.append(serializable_result)

        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(serializable_results, f, indent=2, ensure_ascii=False)

        logger.info(f"Results exported to {filename}")

    def close(self) -> None:
        """
        Close the logger and clean up resources.
        """
        if self.file_handler:
            self.file_handler.close()
            logger.removeHandler(self.file_handler)


def log_retrieval_results(query: str, results: List[Dict], validation_summary: Dict) -> None:
    """
    Log retrieval results with detailed information (function version).

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


def log_performance_metrics(start_time: float, query: str, results_count: int) -> float:
    """
    Log performance metrics for a retrieval operation.

    Args:
        start_time (float): Start time of the operation
        query (str): The query that was executed
        results_count (int): Number of results returned

    Returns:
        float: Execution time in seconds
    """
    execution_time = time.time() - start_time
    logger.info(f"Query '{query[:30]}...' took {execution_time:.4f}s and returned {results_count} results")
    return execution_time


def create_test_report(all_results: List[Dict], edge_case_results: Dict) -> Dict:
    """
    Create a comprehensive test report.

    Args:
        all_results (List[Dict]): Results from all sample queries
        edge_case_results (Dict): Results from edge case testing

    Returns:
        Dict: Comprehensive test report
    """
    total_queries = len(all_results)
    total_chunks = sum(result['validation_summary']['total_chunks'] for result in all_results)
    total_valid_chunks = sum(result['validation_summary']['valid_chunks'] for result in all_results)
    avg_validity_percentage = sum(result['validation_summary']['validity_percentage'] for result in all_results) / total_queries if total_queries > 0 else 0

    report = {
        'timestamp': datetime.now().isoformat(),
        'total_queries_executed': total_queries,
        'total_chunks_retrieved': total_chunks,
        'total_valid_chunks': total_valid_chunks,
        'average_validity_percentage': avg_validity_percentage,
        'edge_case_results': edge_case_results,
        'overall_status': 'success' if avg_validity_percentage >= 70.0 and total_valid_chunks > 0 else 'warning',
        'summary': f"Executed {total_queries} queries, retrieved {total_chunks} chunks, {avg_validity_percentage:.1f}% validity"
    }

    logger.info("=== RETRIEVAL TESTING REPORT ===")
    logger.info(f"Generated at: {report['timestamp']}")
    logger.info(f"Total queries executed: {total_queries}")
    logger.info(f"Total chunks retrieved: {total_chunks}")
    logger.info(f"Total valid chunks: {total_valid_chunks}")
    logger.info(f"Average validity percentage: {avg_validity_percentage:.1f}%")
    logger.info(f"Overall status: {report['overall_status']}")
    logger.info("Edge case results:")
    for test_name, result in edge_case_results.items():
        logger.info(f"  {test_name}: {result.get('status', 'unknown')}")

    return report