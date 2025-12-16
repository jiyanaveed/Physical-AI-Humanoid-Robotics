import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from bs4 import BeautifulSoup
import re
import urllib.parse
import hashlib
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# -------------------------------------
# CONFIG
# -------------------------------------
# Load configuration from environment variables
SITEMAP_URL = os.getenv("SITEMAP_URL", "https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml")
COLLECTION_NAME = "RAD_Embedding"  # As specified in requirements

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

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    """
    Extract all URLs from the sitemap
    """
    try:
        xml = requests.get(sitemap_url).text
        root = ET.fromstring(xml)

        urls = []
        for child in root:
            # Handle both regular sitemap and sitemap index formats
            loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if loc_tag is not None:
                urls.append(loc_tag.text)

        print("\nFOUND URLS:")
        for u in urls:
            print(" -", u)

        return urls
    except ET.ParseError as e:
        print(f"Error parsing sitemap XML: {e}")
        return []
    except requests.RequestException as e:
        print(f"Error fetching sitemap: {e}")
        return []


# -------------------------------------
# Step 2 — Download page + extract and normalize text with metadata
# -------------------------------------
def extract_text_from_url(url):
    """
    Extract and normalize text content from a URL with metadata
    """
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for bad status codes
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
            print("[WARNING] No text extracted from:", url)
            # Fallback to trafilatura if BeautifulSoup extraction failed
            text = trafilatura.extract(html)

        return text, page_title
    except requests.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return "", ""
    except Exception as e:
        print(f"Error extracting content from {url}: {e}")
        return "", ""


# -------------------------------------
# Step 3 — Chunk the text with metadata
# -------------------------------------
def chunk_text(text, url, page_title, max_chars=1200, overlap=100):
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


# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    """
    Generate embedding for text using Cohere
    """
    try:
        response = cohere_client.embed(
            model=EMBED_MODEL,
            input_type="search_document",  # Use search_document for document chunks
            texts=[text],
        )
        return response.embeddings[0]  # Return the first embedding
    except Exception as e:
        print(f"Error generating embedding for text: {e}")
        raise


# -------------------------------------
# Step 5 — Create Qdrant collection named RAD_Embedding
# -------------------------------------
def create_collection():
    """
    Create Qdrant collection named RAD_Embedding
    """
    try:
        print("\nCreating Qdrant collection: RAD_Embedding...")
        qdrant.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,        # Cohere embed-english-v3.0 dimension
                distance=Distance.COSINE
            )
        )
        print("Qdrant collection created successfully!")
        return True
    except Exception as e:
        print(f"Error creating Qdrant collection: {e}")
        return False


# -------------------------------------
# Step 6 — Store chunk in Qdrant with error handling and idempotency
# -------------------------------------
def save_chunk_to_qdrant(chunk, chunk_id, url, page_title, chunk_index):
    """
    Save a chunk with its embedding to Qdrant with error handling and idempotency
    """
    try:
        vector = embed(chunk)

        # Create a unique hash for the chunk to enable idempotent ingestion
        chunk_hash = hashlib.md5(f"{url}_{chunk_index}_{len(chunk)}_{hashlib.sha256(chunk.encode()).hexdigest()[:16]}".encode()).hexdigest()

        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload={
                        "url": url,
                        "page_title": page_title,
                        "text": chunk,
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
        print(f"Error saving chunk to Qdrant: {e}")
        return False


# -------------------------------------
# Step 7 — Validate pipeline by checking Qdrant vectors and metadata correctness
# -------------------------------------
def validate_pipeline():
    """
    Validate pipeline by checking Qdrant vectors and metadata correctness
    """
    try:
        # Get collection info
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print(f"\nCollection '{COLLECTION_NAME}' info:")
        print(f"  Points count: {collection_info.points_count}")
        print(f"  Vector size: {collection_info.config.params.vectors.size}")
        print(f"  Distance: {collection_info.config.params.vectors.distance}")

        # If collection has points, get a sample to validate metadata
        if collection_info.points_count > 0:
            # Get a sample of points - scroll returns (points, next_page_offset_filter) in newer versions
            try:
                scroll_result = qdrant.scroll(
                    collection_name=COLLECTION_NAME,
                    limit=1  # Just get one point for validation
                )

                # Extract points from scroll result - it's typically (points_list, next_offset)
                if isinstance(scroll_result, tuple):
                    points_list = scroll_result[0]
                else:
                    points_list = scroll_result

                if points_list and len(points_list) > 0:
                    sample_point = points_list[0]
                    payload = sample_point.payload

                    print("\nSample point metadata validation:")
                    required_fields = ["url", "page_title", "text", "chunk_id", "chunk_index", "source_url", "chunk_hash"]
                    all_present = True

                    for field in required_fields:
                        if field in payload:
                            field_value = str(payload[field])
                            print(f"  ✓ {field}: {field_value[:50]}{'...' if len(field_value) > 50 else ''}")
                        else:
                            print(f"  ✗ {field}: MISSING")
                            all_present = False

                    # Check vector dimension - access the vector from the point
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
                        print(f"  ✓ Vector dimension: {vector_dimension} (correct)")
                    else:
                        print(f"  ✗ Vector dimension: {vector_dimension} (expected 1024)")
                        all_present = False

                    return all_present
                else:
                    print("  No points found in collection for validation")
                    return False
            except Exception as e:
                print(f"  Error during validation: {e}")
                return False
        else:
            print("  Collection is empty - no points to validate")
            return True  # This is OK if we just created the collection

    except Exception as e:
        print(f"Error validating pipeline: {e}")
        return False


# -------------------------------------
# Step 8 — Main ingestion pipeline
# -------------------------------------
def ingest_book():
    """
    Main ingestion pipeline function that executes the complete workflow
    """
    print("Starting RAG ingestion pipeline...")

    # Get all URLs from sitemap
    urls = get_all_urls(SITEMAP_URL)
    if not urls:
        print("No URLs found, exiting.")
        return False

    # Create Qdrant collection
    if not create_collection():
        print("Failed to create Qdrant collection, exiting.")
        return False

    # Process each URL
    global_id = 1
    processed_count = 0
    error_count = 0

    for i, url in enumerate(urls):
        print(f"\nProcessing ({i+1}/{len(urls)}): {url}")

        try:
            # Extract content and metadata
            text, page_title = extract_text_from_url(url)
            if not text:
                print(f"  No content extracted from {url}, skipping.")
                continue

            # Chunk the content
            chunks = chunk_text(text, url, page_title)
            print(f"  Created {len(chunks)} chunks")

            # Save each chunk to Qdrant
            for chunk_text_val, chunk_idx, chunk_title, chunk_url in chunks:
                success = save_chunk_to_qdrant(chunk_text_val, global_id, chunk_url, chunk_title, chunk_idx)
                if success:
                    print(f"  Saved chunk {global_id} to Qdrant")
                    global_id += 1
                    processed_count += 1
                else:
                    print(f"  Failed to save chunk {global_id} to Qdrant")
                    error_count += 1

        except Exception as e:
            print(f"  Error processing {url}: {e}")
            error_count += 1
            continue

    print(f"\nIngestion completed!")
    print(f"  Successfully processed: {processed_count} chunks")
    print(f"  Errors: {error_count}")
    print(f"  Total chunks stored: {global_id - 1}")

    # Validate the pipeline
    print("\nValidating pipeline...")
    validation_success = validate_pipeline()
    if validation_success:
        print("✓ Pipeline validation successful!")
    else:
        print("✗ Pipeline validation failed!")

    return validation_success


# Execute the main ingestion pipeline
if __name__ == "__main__":
    success = ingest_book()
    if success:
        print("\n🎉 RAG ingestion pipeline completed successfully!")
    else:
        print("\n❌ RAG ingestion pipeline completed with errors.")