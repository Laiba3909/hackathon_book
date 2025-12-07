# src/embeddings_pipeline/embedder.py

import os
from openai import OpenAI
from typing import List
from src.embeddings_pipeline.logger import setup_logging # Import setup_logging

logger = setup_logging(__name__) # Initialize logger


class Embedder:
    def __init__(self):
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            logger.critical("OPENAI_API_KEY environment variable not set.")
            raise ValueError("OPENAI_API_KEY environment variable not set.")
        
        self.client = OpenAI(api_key=self.openai_api_key)
        self.model = "text-embedding-ada-002" # A common OpenAI embedding model
        self.vector_size = 1536 # text-embedding-ada-002 produces 1536-dimensional vectors
        logger.info(f"Embedder initialized with model: {self.model}")

    def get_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding for the given text using OpenAI's API.
        """
        if not text.strip():
            logger.warning("Attempted to get embedding for empty text.")
            return [] # Return empty list for empty text

        try:
            response = self.client.embeddings.create(
                input=[text],
                model=self.model
            )
            logger.debug(f"Successfully generated embedding for text: {text[:50]}...")
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding for text: {text[:50]}... Error: {e}", exc_info=True)
            return [] # Return empty list on error

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a batch of texts.
        """
        if not texts:
            logger.warning("Attempted to get embeddings for an empty list of texts.")
            return []

        # Filter out empty strings to avoid API errors
        non_empty_texts = [text for text in texts if text.strip()]
        if not non_empty_texts:
            logger.warning("All texts in batch were empty after stripping.")
            return [[] for _ in texts] # Return list of empty lists for each original empty text

        try:
            response = self.client.embeddings.create(
                input=non_empty_texts,
                model=self.model
            )
            logger.debug(f"Successfully generated embeddings for a batch of {len(non_empty_texts)} texts.")
            # Map embeddings back to original list, placing empty list for originally empty texts
            embeddings_map = {text: emb for text, emb in zip(non_empty_texts, response.data)}
            result_embeddings = []
            for text in texts:
                if text.strip():
                    result_embeddings.append(embeddings_map[text].embedding)
                else:
                    result_embeddings.append([])
            return result_embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings for batch. Error: {e}", exc_info=True)
            return [[] for _ in texts] # Return list of empty lists on error

# Example usage (for testing)
if __name__ == "__main__":
    logger.info("Running Embedder example usage.")
    # Ensure OPENAI_API_KEY is set in your .env file or environment
    embedder = Embedder()
    
    test_text = "This is a test sentence for embedding."
    embedding = embedder.get_embedding(test_text)
    if embedding:
        logger.info(f"Embedding length: {len(embedding)}")
        logger.info(f"First 5 dimensions: {embedding[:5]}")
    else:
        logger.error("Failed to get embedding for test_text.")

    test_texts_batch = [
        "Hello from batch 1.",
        "Another sentence for batch 2.",
        "", # Test empty string
        "Final sentence for batch 3."
    ]
    embeddings_batch = embedder.get_embeddings_batch(test_texts_batch)
    if embeddings_batch:
        for i, emb in enumerate(embeddings_batch):
            if emb:
                logger.info(f"Batch embedding {i} length: {len(emb)}")
            else:
                logger.warning(f"Batch embedding {i} is empty.")
    else:
        logger.error("Failed to get batch embeddings.")
