"""
Main FastAPI application for the RAG Agent.
"""
import os
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for the RAG (Retrieval-Augmented Generation) agent that queries Qdrant embeddings and returns relevant book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import routers
from src.api.health_router import router as health_router
from src.api.query_router import router as query_router
from src.api.ingestion_router import router as ingestion_router

# Include routers with proper prefixes
app.include_router(health_router, prefix="/health", tags=["health"])
app.include_router(query_router, prefix="/query", tags=["query"])
app.include_router(ingestion_router, prefix="/ingestion", tags=["ingestion"])


@app.on_event("startup")
async def startup_event():
    """Startup event handler."""
    logger.info("Starting up RAG Agent API")
    # Any initialization code can go here

@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler."""
    logger.info("Shutting down RAG Agent API")
    # Any cleanup code can go here

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False)