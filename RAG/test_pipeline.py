"""
Test script to validate the RAG ingestion pipeline
"""
import os
from dotenv import load_dotenv
import requests
from main import get_all_urls, extract_text_from_url, chunk_text, check_if_already_ingested

# Load environment variables
load_dotenv()

def test_sitemap_extraction():
    """Test sitemap URL extraction"""
    print("Testing sitemap extraction...")
    try:
        sitemap_url = os.getenv("SITEMAP_URL", "https://physicalhumanoidaitextbook.vercel.app/sitemap.xml")
        urls = get_all_urls(sitemap_url)
        print(f"✓ Found {len(urls)} URLs from sitemap")
        if urls:
            print(f"  First URL: {urls[0]}")
        return True
    except Exception as e:
        print(f"✗ Sitemap extraction failed: {e}")
        return False

def test_content_extraction():
    """Test content extraction from a sample URL"""
    print("\nTesting content extraction...")
    try:
        # Use the first URL from sitemap or a default one
        sitemap_url = os.getenv("SITEMAP_URL", "https://physicalhumanoidaitextbook.vercel.app/sitemap.xml")
        urls = get_all_urls(sitemap_url)
        test_url = urls[0] if urls else "https://physicalhumanoidaitextbook.vercel.app/"

        text, page_title = extract_text_from_url(test_url)
        print(f"✓ Extracted content from {test_url}")
        print(f"  Page title: {page_title}")
        print(f"  Text length: {len(text)} characters")
        return True
    except Exception as e:
        print(f"✗ Content extraction failed: {e}")
        return False

def test_chunking():
    """Test content chunking with metadata"""
    print("\nTesting content chunking...")
    try:
        # Use a sample text or extract from a URL
        sitemap_url = os.getenv("SITEMAP_URL", "https://physicalhumanoidaitextbook.vercel.app/sitemap.xml")
        urls = get_all_urls(sitemap_url)
        test_url = urls[0] if urls else "https://physicalhumanoidaitextbook.vercel.app/"

        text, page_title = extract_text_from_url(test_url)

        if len(text) > 100:  # Only test if we have enough content
            chunks = chunk_text(text[:2000], test_url, page_title)  # Limit to first 2000 chars for testing
            print(f"✓ Chunked content into {len(chunks)} chunks")
            if chunks:
                print(f"  First chunk: {len(chunks[0][0])} chars, index: {chunks[0][1]}")
            return True
        else:
            print("  Not enough content to test chunking properly")
            return True  # Not a failure, just not enough content
    except Exception as e:
        print(f"✗ Content chunking failed: {e}")
        return False

def test_idempotent_check():
    """Test idempotent ingestion check"""
    print("\nTesting idempotent check...")
    try:
        # This test will show if the function works without error
        result = check_if_already_ingested("https://example.com", 0, 100)
        print(f"✓ Idempotent check function works, result: {result}")
        return True
    except Exception as e:
        print(f"✗ Idempotent check failed: {e}")
        return False

def run_all_tests():
    """Run all tests"""
    print("Running RAG Pipeline Tests\n" + "="*40)

    tests = [
        test_sitemap_extraction,
        test_content_extraction,
        test_chunking,
        test_idempotent_check
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "="*40)
    passed = sum(results)
    total = len(results)
    print(f"Tests passed: {passed}/{total}")

    if passed == total:
        print("✓ All tests passed!")
        return True
    else:
        print(f"✗ {total - passed} test(s) failed")
        return False

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)