# src/embeddings_pipeline/qdrant_client.py

import os
from qdrant_client import QdrantClient, models
from typing import List, Dict
from src.embeddings_pipeline.logger import setup_logging # Import setup_logging

logger = setup_logging(__name__) # Initialize logger

# QDRANT_HOST and QDRANT_API_KEY will be loaded from environment variables
# as load_dotenv() is called in main.py or pipeline.py


QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

class QdrantClientWrapper:
    def __init__(self):
        if not QDRANT_HOST or not QDRANT_API_KEY:
            logger.critical("QDRANT_HOST and QDRANT_API_KEY must be set in the environment variables.")
            raise ValueError("QDRANT_HOST and QDRANT_API_KEY must be set in the environment variables.")
        
        self.client = QdrantClient(
            host=QDRANT_HOST, 
            api_key=QDRANT_API_KEY
        )
        logger.info("QdrantClientWrapper initialized.")

    def health_check(self):
        try:
            health = self.client.health_check()
            logger.info(f"Qdrant health check successful: {health}")
            return {"status": "ok", "health": health}
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}", exc_info=True)
            return {"status": "error", "message": str(e)}

    def get_or_create_collection(self, collection_name: str, vector_size: int, distance_metric: models.Distance = models.Distance.COSINE):
        try:
            if not self.client.collection_exists(collection_name=collection_name).exists:
                logger.info(f"Creating collection '{collection_name}' with vector size {vector_size} and metric {distance_metric.value}...")
                self.client.create_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(size=vector_size, distance=distance_metric),
                )
                logger.info(f"Collection '{collection_name}' created.")
            else:
                logger.info(f"Collection '{collection_name}' already exists.")
            return True
        except Exception as e:
            logger.error(f"Error getting or creating collection '{collection_name}': {e}", exc_info=True)
            return False

    def upsert_vectors(self, collection_name: str, vectors: List[List[float]], payloads: List[Dict], ids: List[int] = None):
        """
        Upserts vectors and their payloads to the specified collection.
        :param collection_name: Name of the collection.
        :param vectors: List of vectors to upsert.
        :param payloads: List of payloads corresponding to each vector.
        :param ids: Optional list of explicit IDs for the points. If None, Qdrant generates them.
        """
        if not vectors or not payloads:
            logger.warning("No vectors or payloads to upsert.")
            return False

        if len(vectors) != len(payloads):
            logger.error("Length of vectors and payloads must be the same.")
            raise ValueError("Length of vectors and payloads must be the same.")

        try:
            operation_info = self.client.upsert(
                collection_name=collection_name,
                wait=True,
                points=models.Batch(
                    ids=ids,
                    vectors=vectors,
                    payloads=payloads,
                ),
            )
            logger.info(f"Upsert operation to '{collection_name}' completed with status: {operation_info.status}")
            return operation_info.status == models.UpdateStatus.COMPLETED
        except Exception as e:
            logger.error(f"Error upserting vectors to '{collection_name}': {e}", exc_info=True)
            return False

    def search_vectors(self, collection_name: str, query_vector: List[float], limit: int = 5, score_threshold: float = 0.5) -> List[models.ScoredPoint]:
        """
        Searches for similar vectors in the specified collection.
        :param collection_name: Name of the collection to search.
        :param query_vector: The vector to search for.
        :param limit: Maximum number of results to return.
        :param score_threshold: Minimum score of results to return.
        :return: A list of ScoredPoint objects.
        """
        if not query_vector:
            logger.warning("Query vector cannot be empty for search.")
            return []

        try:
            search_result = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit,
                query_filter=None, # No additional filters for now
                with_payload=True, # Return the payload along with the vector
                score_threshold=score_threshold
            )
            logger.info(f"Search in '{collection_name}' returned {len(search_result)} results.")
            return search_result
        except Exception as e:
            logger.error(f"Error searching vectors in '{collection_name}': {e}", exc_info=True)
            return []

if __name__ == "__main__":
    logger.info("Running QdrantClientWrapper example usage.")
    # Set QDRANT_HOST and QDRANT_API_KEY in your .env file
    # Or export them before running this script
    
    # Create a dummy .env file for testing purposes if not present
    if not os.path.exists(".env"):
        logger.warning("No .env file found. Creating a dummy one for example usage. Remember to configure your actual .env file.")
        with open(".env", "w") as f:
            f.write("QDRANT_HOST=http://localhost:6333\n") # Replace with your test Qdrant host
            f.write("QDRANT_API_KEY=test_api_key\n") # Replace with your test API key
            f.write("OPENAI_API_KEY=sk-test-key\n") # Placeholder for embedder testing
    
    from dotenv import load_dotenv
    load_dotenv() # Load for example usage if .env was just created

    qdrant_wrapper = QdrantClientWrapper()
    health_status = qdrant_wrapper.health_check()
    logger.info(f"Qdrant Health Check: {health_status}")

    # Example of collection creation and upsert (requires a running Qdrant instance)
    test_collection_name = "test_collection_pipeline"
    test_vector_size = 1536 # Example size for text-embedding-ada-002

    if qdrant_wrapper.get_or_create_collection(test_collection_name, test_vector_size):
        # Dummy data for upsert
        sample_vectors = [[0.1] * test_vector_size, [0.2] * test_vector_size]
        sample_payloads = [{"text": "Hello world", "source": "test"}, {"text": "Goodbye world", "source": "test"}]
        
        if qdrant_wrapper.upsert_vectors(test_collection_name, sample_vectors, sample_payloads):
            logger.info("Sample vectors upserted successfully.")
        else:
            logger.error("Failed to upsert sample vectors.")

    # Example search (requires some vectors to be upserted)
    # This requires an Embedder instance to get a query vector
    # from src.embeddings_pipeline.embedder import Embedder
    # embedder = Embedder()
    # query_text = "What is robotics?"
    # query_vector = embedder.get_embedding(query_text)
    # if query_vector:
    #     search_results = qdrant_wrapper.search_vectors(test_collection_name, query_vector, limit=2)
    #     if search_results:
    #         logger.info("\nSearch Results:")
    #         for hit in search_results:
    #             logger.info(f"ID: {hit.id}, Score: {hit.score}, Payload: {hit.payload}")
    #     else:
    #         logger.info("\nNo search results.")
    # else:
    #     logger.error("\nFailed to get query embedding for search example.")
