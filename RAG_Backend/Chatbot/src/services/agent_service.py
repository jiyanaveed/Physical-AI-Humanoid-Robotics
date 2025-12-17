"""
Agent service for the RAG Agent API that integrates with OpenAI Agent SDK.
"""
import os
import logging
from typing import Dict, Any, Optional
from openai import OpenAI
from ..config import config

# Configure logging
logger = logging.getLogger(__name__)

class AgentService:
    """Service class to handle OpenAI Agent SDK integration for query processing."""

    def __init__(self):
        """Initialize the Agent service with OpenAI client."""
        if not config.OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = OpenAI(api_key=config.OPENAI_API_KEY)
        logger.info("Agent service initialized with OpenAI client")

    async def process_query(self, query_text: str) -> Dict[str, Any]:
        """
        Process a user query using the OpenAI Agent SDK.

        Args:
            query_text (str): The user's question to process

        Returns:
            Dict[str, Any]: Structured response with relevant information
        """
        try:
            logger.info(f"Processing query with OpenAI: {query_text[:50]}...")

            # For now, we'll use the chat completions API as a placeholder
            # In a real implementation, this would use the specific Agent SDK features
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",  # This would be configurable in a real implementation
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that can answer questions based on provided context."},
                    {"role": "user", "content": query_text}
                ],
                max_tokens=500,
                temperature=0.7
            )

            result = {
                "query": query_text,
                "response": response.choices[0].message.content,
                "model": response.model,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens
                }
            }

            logger.info(f"Query processed successfully: {query_text[:50]}...")
            return result

        except Exception as e:
            logger.error(f"Error processing query '{query_text[:50]}...': {str(e)}")
            raise

# Global agent service instance - defer initialization to avoid startup issues
def _get_agent_service():
    """Lazy initialization of agent service to avoid startup errors."""
    try:
        return AgentService()
    except Exception as e:
        logger.warning(f"Agent service failed to initialize: {e}")
        # Return a mock service that logs the issue but doesn't break the app
        class MockAgentService:
            async def process_query(self, query_text: str) -> dict:
                logger.warning("Agent service not available, returning mock response")
                return {
                    "query": query_text,
                    "response": "Mock response - Agent service not available",
                    "model": "mock-model",
                    "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}
                }
        return MockAgentService()

agent_service = _get_agent_service()