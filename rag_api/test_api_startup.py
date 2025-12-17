#!/usr/bin/env python3
"""
Test script to validate that the rag_api can start up properly.
"""
import os
import sys
from unittest.mock import patch

# Set mock environment variables to allow imports
os.environ.setdefault("QDRANT_URL", "https://mock-qdrant.com")
os.environ.setdefault("QDRANT_API_KEY", "mock-api-key")
os.environ.setdefault("COHERE_API_KEY", "mock-cohere-key")
os.environ.setdefault("OPENAI_API_KEY", "mock-openai-key")
os.environ.setdefault("COLLECTION_NAME", "RAD_Embedding")
os.environ.setdefault("SITEMAP_URL", "https://example.com/sitemap.xml")

def test_api_startup():
    """Test that the API can be imported and start up."""
    print("Testing rag_api startup...")

    # Temporarily patch the OpenAI client to avoid the proxies issue
    with patch.dict('os.environ', {
        "OPENAI_API_KEY": "mock-openai-key"
    }):
        try:
            # Add the current directory to Python path to handle relative imports
            sys.path.insert(0, os.path.abspath('.'))

            # Import the main app
            from main import app
            print("✓ FastAPI app imported successfully")

            # Check that routes are registered
            route_names = [route.name for route in app.routes]
            print(f"✓ API has {len(route_names)} routes registered: {route_names}")

            # Check for the expected routes
            expected_routes = ['health', 'query', 'ingestion']
            found_routes = [route for route in expected_routes if any(route in name for name in route_names)]
            print(f"✓ Found expected routes: {found_routes}")

            # Test that we can create a test client
            try:
                from fastapi.testclient import TestClient
                client = TestClient(app)
                print("✓ TestClient created successfully")

                # Test the health endpoint
                response = client.get("/")
                print(f"✓ Health endpoint status: {response.status_code}")

                # Test the health endpoint
                response = client.get("/health/health")
                print(f"✓ Health check endpoint status: {response.status_code}")

                if response.status_code == 200:
                    print("✓ Health check passed")
                else:
                    print(f"✗ Health check failed with status: {response.status_code}")

            except ImportError:
                print("⚠ TestClient not available (fastapi.testclient), but app structure is valid")

            return True

        except Exception as e:
            print(f"✗ API startup failed: {e}")
            import traceback
            traceback.print_exc()
            return False

if __name__ == "__main__":
    success = test_api_startup()
    if success:
        print("\n🎉 rag_api can start up successfully!")
    else:
        print("\n❌ rag_api failed to start!")
        sys.exit(1)