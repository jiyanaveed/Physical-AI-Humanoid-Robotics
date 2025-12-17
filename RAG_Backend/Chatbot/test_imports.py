#!/usr/bin/env python3
"""
Test script to validate that rag_api modules can be imported without errors.
"""
import sys
import os

# Set mock environment variables to allow imports
os.environ.setdefault("QDRANT_URL", "https://mock-qdrant.com")
os.environ.setdefault("QDRANT_API_KEY", "mock-api-key")
os.environ.setdefault("COHERE_API_KEY", "mock-cohere-key")
os.environ.setdefault("OPENAI_API_KEY", "mock-openai-key")

sys.path.insert(0, os.path.abspath('.'))

def test_imports():
    """Test that all rag_api modules can be imported."""
    print("Testing rag_api module imports...")

    # Test config import
    try:
        from src.config import config
        print("✓ Config module imported successfully")
    except Exception as e:
        print(f"✗ Config module import failed: {e}")
        return False

    # Test models import
    try:
        from src.models.content_chunk import ContentChunk
        print("✓ ContentChunk model imported successfully")
    except Exception as e:
        print(f"✗ ContentChunk model import failed: {e}")
        return False

    try:
        from src.models.query import QueryRequest, QueryResponse
        print("✓ Query models imported successfully")
    except Exception as e:
        print(f"✗ Query models import failed: {e}")
        return False

    try:
        from src.models.response import APIResponse, ErrorResponse
        print("✓ Response models imported successfully")
    except Exception as e:
        print(f"✗ Response models import failed: {e}")
        return False

    # Test utility functions
    try:
        from src.utils import setup_logging
        print("✓ Utils module imported successfully")
    except Exception as e:
        print(f"✗ Utils module import failed: {e}")
        # This might not exist, so we'll continue

    # Test logging service
    try:
        from src.services.logging_service import logging_service
        print("✓ Logging service imported successfully")
    except Exception as e:
        print(f"✗ Logging service import failed: {e}")
        return False

    # Test qdrant service (without initializing clients)
    try:
        import src.services.qdrant_service
        print("✓ Qdrant service module imported successfully (note: clients not initialized due to missing API keys)")
    except Exception as e:
        print(f"✗ Qdrant service import failed: {e}")
        return False

    # Test ingestion service (without initializing clients)
    try:
        import src.services.ingestion_service
        print("✓ Ingestion service module imported successfully (note: clients not initialized due to missing API keys)")
    except Exception as e:
        print(f"✗ Ingestion service import failed: {e}")
        return False

    # Test agent service
    try:
        from src.services.agent_service import agent_service
        print("✓ Agent service imported successfully")
    except Exception as e:
        print(f"✗ Agent service import failed: {e}")
        return False

    # Test API routers
    try:
        from src.api.health_router import router as health_router
        print("✓ Health router imported successfully")
    except Exception as e:
        print(f"✗ Health router import failed: {e}")
        return False

    try:
        from src.api.query_router import router as query_router
        print("✓ Query router imported successfully")
    except Exception as e:
        print(f"✗ Query router import failed: {e}")
        return False

    try:
        from src.api.ingestion_router import router as ingestion_router
        print("✓ Ingestion router imported successfully")
    except Exception as e:
        print(f"✗ Ingestion router import failed: {e}")
        return False

    print("\n✓ All modules imported successfully!")
    return True

if __name__ == "__main__":
    success = test_imports()
    if success:
        print("\n🎉 All rag_api modules can be imported successfully!")
    else:
        print("\n❌ Some modules failed to import!")
        sys.exit(1)