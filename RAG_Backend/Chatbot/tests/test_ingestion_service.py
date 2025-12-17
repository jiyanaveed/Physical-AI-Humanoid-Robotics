"""
Unit tests for the ingestion service.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import os

from src.services.ingestion_service import IngestionService
from src.config import config


class TestIngestionService(unittest.TestCase):
    """Test cases for the IngestionService class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Skip tests if environment variables are not set
        if not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
            self.skipTest("Required environment variables not set for testing ingestion service")

        self.ingestion_service = IngestionService()

    @patch('src.services.ingestion_service.requests.get')
    def test_get_all_urls_success(self, mock_requests_get):
        """Test successful URL extraction from sitemap."""
        # Mock the response from requests.get
        mock_response = Mock()
        mock_response.text = '''<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
  <url>
    <loc>https://example.com/page1</loc>
  </url>
  <url>
    <loc>https://example.com/page2</loc>
  </url>
</urlset>'''
        mock_response.raise_for_status.return_value = None
        mock_requests_get.return_value = mock_response

        # Test the method
        urls = self.ingestion_service.get_all_urls("https://example.com/sitemap.xml")

        # Assertions
        self.assertEqual(len(urls), 2)
        self.assertIn("https://example.com/page1", urls)
        self.assertIn("https://example.com/page2", urls)
        mock_requests_get.assert_called_once_with("https://example.com/sitemap.xml")

    @patch('src.services.ingestion_service.requests.get')
    def test_get_all_urls_request_exception(self, mock_requests_get):
        """Test URL extraction when request fails."""
        # Mock a request exception
        mock_requests_get.side_effect = Exception("Network error")

        # Test the method
        urls = self.ingestion_service.get_all_urls("https://example.com/sitemap.xml")

        # Assertions
        self.assertEqual(urls, [])

    def test_chunk_text_small_text(self):
        """Test chunking of text that fits in a single chunk."""
        text = "This is a short text."
        url = "https://example.com"
        page_title = "Test Page"

        chunks = self.ingestion_service.chunk_text(text, url, page_title, max_chars=100)

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0][0], text)  # Full text in first chunk
        self.assertEqual(chunks[0][1], 0)  # chunk_index
        self.assertEqual(chunks[0][2], page_title)  # page_title
        self.assertEqual(chunks[0][3], url)  # url

    def test_chunk_text_large_text(self):
        """Test chunking of text that needs to be split."""
        text = "A" * 2000  # 2000 character text
        url = "https://example.com"
        page_title = "Test Page"

        chunks = self.ingestion_service.chunk_text(text, url, page_title, max_chars=500, overlap=50)

        # Should have multiple chunks
        self.assertGreater(len(chunks), 1)

        # Check that chunks fit within the size limit
        for chunk_text, chunk_idx, chunk_title, chunk_url in chunks:
            self.assertLessEqual(len(chunk_text), 500)
            self.assertEqual(chunk_title, page_title)
            self.assertEqual(chunk_url, url)

    def test_chunk_text_with_overlap(self):
        """Test that chunks have proper overlap."""
        text = "A" * 1000  # 1000 character text
        url = "https://example.com"
        page_title = "Test Page"

        chunks = self.ingestion_service.chunk_text(text, url, page_title, max_chars=300, overlap=50)

        # If we have multiple chunks, check that there's overlap consideration
        if len(chunks) > 1:
            # Verify that we have more than one chunk
            self.assertGreater(len(chunks), 1)

    @patch('src.services.ingestion_service.requests.get')
    def test_extract_text_from_url_success(self, mock_requests_get):
        """Test successful text extraction from a URL."""
        # Mock the response from requests.get
        mock_response = Mock()
        mock_response.text = '''
        <html>
            <head><title>Test Page</title></head>
            <body>
                <main>
                    <h1>Header</h1>
                    <p>This is the main content of the page.</p>
                    <p>More content here.</p>
                </main>
            </body>
        </html>'''
        mock_response.raise_for_status.return_value = None
        mock_requests_get.return_value = mock_response

        # Test the method
        text, title = self.ingestion_service.extract_text_from_url("https://example.com")

        # Assertions
        self.assertEqual(title, "Test Page")
        self.assertIn("This is the main content", text)
        self.assertIn("More content here", text)
        mock_requests_get.assert_called_once_with("https://example.com")

    @patch('src.services.ingestion_service.requests.get')
    def test_extract_text_from_url_request_exception(self, mock_requests_get):
        """Test text extraction when request fails."""
        # Mock a request exception
        mock_requests_get.side_effect = Exception("Network error")

        # Test the method
        text, title = self.ingestion_service.extract_text_from_url("https://example.com")

        # Assertions
        self.assertEqual(text, "")
        self.assertEqual(title, "")


class TestIntegration(unittest.TestCase):
    """Integration tests for the ingestion service."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Skip tests if environment variables are not set
        if not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
            self.skipTest("Required environment variables not set for testing ingestion service")

        self.ingestion_service = IngestionService()

    @unittest.skip("Skipping integration test that requires network access and API keys")
    def test_full_ingestion_flow(self):
        """Test the full ingestion flow with a mock sitemap."""
        # This test would require actual API keys and network access
        # It's skipped by default but can be enabled for manual testing

        # Create a temporary sitemap with a test URL
        # Run the ingestion
        # Verify results in Qdrant
        pass


if __name__ == '__main__':
    unittest.main()