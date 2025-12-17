"""
Configuration module for the RAG Agent API.
"""
import os
from typing import Optional

class Config:
    """Configuration class to store environment variables and constants."""

    # OpenAI Configuration
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")

    # Qdrant Configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")

    # Cohere Configuration
    COHERE_API_KEY: Optional[str] = os.getenv("COHERE_API_KEY")

    # App Configuration
    PORT: int = int(os.getenv("PORT", 8000))
    COLLECTION_NAME: str = os.getenv("COLLECTION_NAME", "RAD_Embedding")
    SITEMAP_URL: str = os.getenv("SITEMAP_URL", "https://physical-ai-humanoid-robotics-git-main-javeria-naveeds-projects.vercel.app/sitemap.xml")

    # Validation
    @classmethod
    def validate(cls) -> bool:
        """Validate that all required environment variables are set."""
        required_vars = [
            cls.QDRANT_URL,
            cls.QDRANT_API_KEY,
            cls.COHERE_API_KEY
        ]

        for var in required_vars:
            if not var:
                return False
        return True

# Create global config instance
config = Config()