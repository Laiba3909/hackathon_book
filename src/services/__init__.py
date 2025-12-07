# src/services/__init__.py

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from src.embeddings_pipeline.qdrant_client import QdrantClientWrapper # Assuming it's importable

# Load environment variables
load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY") # Although not directly used here, good to load

# --- Environment Variable Validation ---
if not QDRANT_HOST:
    raise ValueError("QDRANT_HOST environment variable not set.")
if not QDRANT_API_KEY:
    raise ValueError("QDRANT_API_KEY environment variable not set.")
if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable not set.")


# --- Initialize Qdrant Client for reuse ---
# We use the QdrantClientWrapper from the embeddings pipeline
# to ensure consistency and reuse the health check and other methods.
try:
    qdrant_client_wrapper = QdrantClientWrapper()
except ValueError as e:
    # Re-raise with a more informative message if Qdrant env vars are missing
    raise ValueError(f"Failed to initialize Qdrant client in services: {e}")


# --- Service Health Check ---
def get_service_health():
    qdrant_health = qdrant_client_wrapper.health_check()
    openai_status = "OK" if OPENAI_API_KEY else "ERROR: OPENAI_API_KEY not set" # Simple check for now
    
    overall_status = "OK" if qdrant_health.get("status") == "ok" and openai_status == "OK" else "DEGRADED"

    return {
        "status": overall_status,
        "qdrant": qdrant_health,
        "openai": {"status": openai_status}
    }
