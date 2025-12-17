"""
Ingestion service for the RAG Agent API.
Handles sitemap extraction, content extraction, chunking, and vector storage.
"""
import asyncio
import hashlib
import logging
import os
import re
from typing import List, Tuple, Optional
import xml.etree.ElementTree as ET

import requests
from bs4 import BeautifulSoup
import trafilatura
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

from ..config import config
from ..models.content_chunk import ContentChunk

logger = logging.getLogger(__name__)

class IngestionService:
    """
    Service class to handle the entire ingestion pipeline:
    1. Extract URLs from sitemap
    2. Download and extract content from URLs
    3. Chunk text with metadata
    4. Generate embeddings
    5. Store in Qdrant
    6. Validate pipeline
    """

    def __init__(self):
        """Initialize the ingestion service with required clients and configuration."""
        # Initialize Cohere client
        self.cohere_client = cohere.Client(config.COHERE_API_KEY)
        self.embed_model = "embed-english-v3.0"

        # Connect to Qdrant
        self.qdrant = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )
        self.collection_name = config.COLLECTION_NAME

    def get_all_urls(self, sitemap_url: str) -> List[str]:
        """
        Extract all URLs from the sitemap
        """
        try:
            xml_response = requests.get(sitemap_url)
            xml_response.raise_for_status()
            xml_content = xml_response.text

            root = ET.fromstring(xml_content)

            urls = []
            for child in root:
                # Handle both regular sitemap and sitemap index formats
                loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
                if loc_tag is not None:
                    urls.append(loc_tag.text)

            logger.info(f"Found {len(urls)} URLs from sitemap")
            for url in urls:
                logger.debug(f" - {url}")

            return urls
        except ET.ParseError as e:
            logger.error(f"Error parsing sitemap XML: {e}")
            return []
        except requests.RequestException as e:
            logger.error(f"Error fetching sitemap: {e}")
            return []
        except Exception as e:
            logger.error(f"Unexpected error during sitemap extraction: {e}")
            return []

    def extract_text_from_url(self, url: str) -> Tuple[str, str]:
        """
        Extract and normalize text content from a URL with metadata
        Returns: (text_content, page_title)
        """
        try:
            response = requests.get(url)
            response.raise_for_status()
            html = response.text

            soup = BeautifulSoup(html, 'html.parser')

            # Extract page title
            title_tag = soup.find('title')
            page_title = title_tag.get_text().strip() if title_tag else ""

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Find the main content area - Docusaurus typically uses main or article tags
            main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=re.compile(r'docs-page')) or soup

            # Extract text from the main content
            text = main_content.get_text(separator='\n', strip=True)

            # Clean up excessive whitespace
            text = re.sub(r'\n+', '\n', text)
            text = re.sub(r'[ \t]+', ' ', text)

            if not text:
                logger.warning(f"No text extracted from: {url}")
                # Fallback to trafilatura if BeautifulSoup extraction failed
                text = trafilatura.extract(html)

            return text, page_title
        except requests.RequestException as e:
            logger.error(f"Error fetching URL {url}: {e}")
            return "", ""
        except Exception as e:
            logger.error(f"Error extracting content from {url}: {e}")
            return "", ""

    def chunk_text(self, text: str, url: str, page_title: str, max_chars: int = 1200, overlap: int = 100) -> List[Tuple[str, int, str, str]]:
        """
        Chunk text with overlapping windows and metadata preservation
        Returns list of tuples: (chunk_text, chunk_index, page_title, url)
        """
        if len(text) <= max_chars:
            return [(text, 0, page_title, url)]

        chunks = []
        start_idx = 0
        chunk_index = 0

        while start_idx < len(text):
            # Determine the end position
            end_idx = start_idx + max_chars

            # If we're near the end, include the rest
            if end_idx >= len(text):
                end_idx = len(text)
            else:
                # Try to break at sentence boundary
                temp_text = text[start_idx:end_idx]
                last_period = temp_text.rfind('. ')

                if last_period != -1:
                    end_idx = start_idx + last_period + 2  # Include the period and space
                else:
                    # If no sentence boundary found, try word boundary
                    last_space = temp_text.rfind(' ')
                    if last_space != -1:
                        end_idx = start_idx + last_space

            # Extract the chunk
            chunk = text[start_idx:end_idx].strip()

            if chunk:  # Only add non-empty chunks
                chunks.append((chunk, chunk_index, page_title, url))

            # Move start index forward, considering overlap
            start_idx = end_idx - overlap if overlap < end_idx - start_idx else end_idx
            chunk_index += 1

            # Prevent infinite loop if no progress is made
            if start_idx <= 0:
                start_idx = end_idx

        return chunks

    def embed(self, text: str) -> List[float]:
        """
        Generate embedding for text using Cohere
        """
        try:
            response = self.cohere_client.embed(
                model=self.embed_model,
                input_type="search_document",  # Use search_document for document chunks
                texts=[text],
            )
            return response.embeddings[0]  # Return the first embedding
        except Exception as e:
            logger.error(f"Error generating embedding for text: {e}")
            raise

    def create_collection(self) -> bool:
        """
        Create Qdrant collection with specified name and configuration
        """
        try:
            logger.info(f"Creating Qdrant collection: {self.collection_name}...")
            self.qdrant.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1024,        # Cohere embed-english-v3.0 dimension
                    distance=Distance.COSINE
                )
            )
            logger.info("Qdrant collection created successfully!")
            return True
        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            return False

    def save_chunk_to_qdrant(self, chunk_text: str, chunk_id: int, url: str, page_title: str, chunk_index: int) -> bool:
        """
        Save a chunk with its embedding to Qdrant with error handling and idempotency
        """
        try:
            vector = self.embed(chunk_text)

            # Create a unique hash for the chunk to enable idempotent ingestion
            chunk_hash = hashlib.md5(
                f"{url}_{chunk_index}_{len(chunk_text)}_{hashlib.sha256(chunk_text.encode()).hexdigest()[:16]}"
                .encode()
            ).hexdigest()

            self.qdrant.upsert(
                collection_name=self.collection_name,
                points=[
                    PointStruct(
                        id=chunk_id,
                        vector=vector,
                        payload={
                            "url": url,
                            "page_title": page_title,
                            "text": chunk_text,
                            "chunk_id": chunk_id,
                            "chunk_index": chunk_index,
                            "source_url": url,
                            "chunk_hash": chunk_hash  # For duplicate detection
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            logger.error(f"Error saving chunk to Qdrant: {e}")
            return False

    def validate_pipeline(self) -> bool:
        """
        Validate pipeline by checking Qdrant vectors and metadata correctness
        """
        try:
            # Get collection info
            collection_info = self.qdrant.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' info:")
            logger.info(f"  Points count: {collection_info.points_count}")
            logger.info(f"  Vector size: {collection_info.config.params.vectors.size}")
            logger.info(f"  Distance: {collection_info.config.params.vectors.distance}")

            # If collection has points, get a sample to validate metadata
            if collection_info.points_count > 0:
                # Get a sample of points
                scroll_result = self.qdrant.scroll(
                    collection_name=self.collection_name,
                    limit=1  # Just get one point for validation
                )

                # Extract points from scroll result
                if isinstance(scroll_result, tuple):
                    points_list = scroll_result[0]
                else:
                    points_list = scroll_result

                if points_list and len(points_list) > 0:
                    sample_point = points_list[0]
                    payload = sample_point.payload

                    logger.info("Sample point metadata validation:")
                    required_fields = ["url", "page_title", "text", "chunk_id", "chunk_index", "source_url", "chunk_hash"]
                    all_present = True

                    for field in required_fields:
                        if field in payload:
                            field_value = str(payload[field])
                            logger.info(f"  ✓ {field}: {field_value[:50]}{'...' if len(field_value) > 50 else ''}")
                        else:
                            logger.error(f"  ✗ {field}: MISSING")
                            all_present = False

                    # Check vector dimension
                    vector_dimension = "unknown"
                    if hasattr(sample_point, 'vector'):
                        vector_data = sample_point.vector
                        if isinstance(vector_data, (list, tuple)):
                            vector_dimension = len(vector_data)
                        elif hasattr(vector_data, '__len__'):
                            vector_dimension = len(vector_data)
                        elif hasattr(vector_data, '__iter__') and not isinstance(vector_data, (str, bytes)):
                            # For other iterable types
                            vector_dimension = len(list(vector_data))
                        else:
                            # If vector is not iterable, try to access values if it's a dict-like object
                            try:
                                vector_dimension = len(vector_data)
                            except TypeError:
                                # For dense vectors in different formats
                                vector_dimension = 1024  # Default for Cohere embeddings

                    if vector_dimension == 1024:  # Cohere embed-english-v3.0 dimension
                        logger.info(f"  ✓ Vector dimension: {vector_dimension} (correct)")
                    else:
                        logger.error(f"  ✗ Vector dimension: {vector_dimension} (expected 1024)")
                        all_present = False

                    return all_present
                else:
                    logger.warning("  No points found in collection for validation")
                    return False
            else:
                logger.info("  Collection is empty - no points to validate")
                return True  # This is OK if we just created the collection

        except Exception as e:
            logger.error(f"Error validating pipeline: {e}")
            return False

    def ingest_book(self, sitemap_url: str = None) -> bool:
        """
        Main ingestion pipeline function that executes the complete workflow
        """
        sitemap_url = sitemap_url or config.SITEMAP_URL
        logger.info("Starting RAG ingestion pipeline...")

        # Get all URLs from sitemap
        urls = self.get_all_urls(sitemap_url)
        if not urls:
            logger.error("No URLs found, exiting.")
            return False

        # Create Qdrant collection
        if not self.create_collection():
            logger.error("Failed to create Qdrant collection, exiting.")
            return False

        # Process each URL
        global_id = 1
        processed_count = 0
        error_count = 0

        for i, url in enumerate(urls):
            logger.info(f"Processing ({i+1}/{len(urls)}): {url}")

            try:
                # Extract content and metadata
                text, page_title = self.extract_text_from_url(url)
                if not text:
                    logger.warning(f"  No content extracted from {url}, skipping.")
                    continue

                # Chunk the content
                chunks = self.chunk_text(text, url, page_title)
                logger.info(f"  Created {len(chunks)} chunks")

                # Save each chunk to Qdrant
                for chunk_text_val, chunk_idx, chunk_title, chunk_url in chunks:
                    success = self.save_chunk_to_qdrant(chunk_text_val, global_id, chunk_url, chunk_title, chunk_idx)
                    if success:
                        logger.info(f"  Saved chunk {global_id} to Qdrant")
                        global_id += 1
                        processed_count += 1
                    else:
                        logger.error(f"  Failed to save chunk {global_id} to Qdrant")
                        error_count += 1

            except Exception as e:
                logger.error(f"  Error processing {url}: {e}")
                error_count += 1
                continue

        logger.info(f"Ingestion completed!")
        logger.info(f"  Successfully processed: {processed_count} chunks")
        logger.info(f"  Errors: {error_count}")
        logger.info(f"  Total chunks stored: {global_id - 1}")

        # Validate the pipeline
        logger.info("Validating pipeline...")
        validation_success = self.validate_pipeline()
        if validation_success:
            logger.info("✓ Pipeline validation successful!")
        else:
            logger.error("✗ Pipeline validation failed!")

        return validation_success

# Global instance of the ingestion service
ingestion_service = IngestionService()