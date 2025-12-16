import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from qdrant_client.http import models as qdrant_models
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
SITEMAP_URL = os.getenv("SITEMAP_URL", "https://physicalhumanoidaitextbook.vercel.app/sitemap.xml")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "humanoid_ai_book")

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
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls


# -------------------------------------
# Step 2 — Download page + extract text with metadata
# -------------------------------------
def extract_text_from_url(url):
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


# -------------------------------------
# Step 3 — Chunk the text with metadata
# -------------------------------------
def chunk_text(text, url, page_title, max_chars=1200, overlap=100):
    """
    Chunk text with overlapping windows and metadata preservation
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
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_document",  # Use search_document for document chunks
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
        size=1024,        # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url, page_title, chunk_index):
    vector = embed(chunk)

    # Create a unique hash for the chunk to enable idempotent ingestion
    import hashlib
    chunk_hash = hashlib.md5(f"{url}_{chunk_index}_{len(chunk)}_{hashlib.sha256(chunk.encode()).hexdigest()[:16]}".encode()).hexdigest()

    # Use the hash as a deterministic ID to prevent duplicates
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
                    "source_url": url,  # Added for clarity
                    "chunk_hash": chunk_hash  # For duplicate detection
                }
            )
        ]
    )


def check_if_already_ingested(url, chunk_index, chunk_length):
    """
    Check if a specific chunk from a URL has already been ingested
    by querying for the combination of URL and chunk_index
    """
    try:
        # Search for points with the same URL and chunk_index
        search_result = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            scroll_filter=qdrant_models.Filter(
                must=[
                    qdrant_models.FieldCondition(
                        key="url",
                        match=qdrant_models.MatchValue(value=url)
                    ),
                    qdrant_models.FieldCondition(
                        key="chunk_index",
                        match=qdrant_models.MatchValue(value=chunk_index)
                    )
                ]
            ),
            limit=1
        )

        # If any result is found, it means this chunk was already ingested
        return len(search_result[0]) > 0
    except Exception as e:
        print(f"Error checking if already ingested: {e}")
        # If there's an error, assume it's not ingested to be safe
        return False


# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)

    create_collection()

    global_id = 1

    for url in urls:
        print("\nProcessing:", url)
        text, page_title = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text, url, page_title)

        for chunk, chunk_index, chunk_page_title, chunk_url in chunks:
            # Check if this chunk has already been ingested
            if check_if_already_ingested(chunk_url, chunk_index, len(chunk)):
                print(f"Skipping chunk {chunk_index} from {url} (already ingested)")
                continue

            save_chunk_to_qdrant(chunk, global_id, chunk_url, chunk_page_title, chunk_index)
            print(f"Saved chunk {global_id} from {url}")
            global_id += 1

    print("\n✔️ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)


if __name__ == "__main__":
    ingest_book()