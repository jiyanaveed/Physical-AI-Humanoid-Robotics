"""
Mock Agent service for the RAG Agent API that works without external dependencies.
"""
import os
import logging
from typing import Dict, Any, Optional

# Configure logging
logger = logging.getLogger(__name__)

class MockAgentService:
    """Mock service class to handle query processing without external dependencies."""

    def __init__(self):
        """Initialize the mock Agent service."""
        logger.info("Mock Agent service initialized")

    async def process_query(self, query_text: str) -> Dict[str, Any]:
        """
        Process a user query using mock responses.

        Args:
            query_text (str): The user's question to process

        Returns:
            Dict[str, Any]: Structured response with relevant information
        """
        try:
            logger.info(f"Processing mock query: {query_text[:50]}...")

            # Create a mock response based on the query
            if "hi" in query_text.lower() or "hello" in query_text.lower():
                response_text = "Hello! I'm the Physical AI and Humanoid Robotics assistant. I can help you with questions about ROS 2 fundamentals, AI algorithms, control systems, and humanoid dynamics. What would you like to know?"
            elif "module" in query_text.lower() or "book" in query_text.lower():
                response_text = "The book is organized into four modules: 1) ROS 2 Fundamentals, 2) Core AI Algorithms, 3) Control and Planning, and 4) Advanced Kinematics & Dynamics. Which module would you like to know more about?"
            elif "robot" in query_text.lower() or "humanoid" in query_text.lower():
                response_text = "Humanoid robotics involves creating robots with human-like characteristics. This includes understanding kinematics, dynamics, control systems, and AI algorithms for locomotion and interaction. Our book covers these topics in detail."
            else:
                response_text = f"I can help you with questions about Physical AI and Humanoid Robotics. Based on your query '{query_text}', I recommend checking out the relevant sections in our documentation. The book covers ROS 2, AI algorithms, control systems, and dynamics."

            result = {
                "query": query_text,
                "response": response_text,
                "model": "mock-model",
                "usage": {
                    "prompt_tokens": 0,
                    "completion_tokens": 0,
                    "total_tokens": 0
                }
            }

            logger.info(f"Mock query processed successfully: {query_text[:50]}...")
            return result

        except Exception as e:
            logger.error(f"Error processing mock query '{query_text[:50]}...': {str(e)}")
            raise

# Global mock agent service instance
def _get_agent_service():
    """Lazy initialization of agent service to handle missing dependencies."""
    try:
        # Try to initialize the real service
        from .agent_service import AgentService
        return AgentService()
    except Exception as e:
        logger.warning(f"Real Agent service failed to initialize: {e}")
        # Return mock service that works without external dependencies
        return MockAgentService()

agent_service = _get_agent_service()