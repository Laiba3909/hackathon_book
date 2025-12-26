# src/embeddings_pipeline/embedder.py

import os
import google.generativeai as genai
from typing import List
from src.embeddings_pipeline.logger import setup_logging

logger = setup_logging(__name__)

class Embedder:
    """
    A wrapper for Google's Gemini API to generate embeddings.
    """
    def __init__(self):
        """
        Initializes the Embedder, configures the Gemini client.
        """
        self.google_api_key = os.getenv("GOOGLE_API_KEY")
        if not self.google_api_key:
            logger.critical("GOOGLE_API_KEY environment variable not set.")
            raise ValueError("GOOGLE_API_KEY environment variable not set.")
        
        # Configuring the SDK with the API key is now done in main.py,
        # so we just need to ensure the key exists.
        
        self.model = "models/embedding-001"  # The recommended model for text embeddings
        # Gemini's `embedding-001` model produces 768-dimensional vectors.
        self.vector_size = 768
        logger.info(f"Embedder initialized with Google Gemini model: {self.model}")

    def get_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding for the given text using Google's Gemini API.
        """
        if not text.strip():
            logger.warning("Attempted to get embedding for empty text.")
            return []

        try:
            # Use genai.embed_content for generating embeddings
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="RETRIEVAL_DOCUMENT" # Use 'RETRIEVAL_QUERY' for user queries
            )
            logger.debug(f"Successfully generated embedding for text: {text[:50]}...")
            return result['embedding']
        except Exception as e:
            logger.error(f"Error generating embedding for text: {text[:50]}... Error: {e}", exc_info=True)
            return []

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a batch of texts using Google's Gemini API.
        """
        if not texts:
            logger.warning("Attempted to get embeddings for an empty list of texts.")
            return []

        # Filter out empty strings
        non_empty_texts = [text for text in texts if text.strip()]
        if not non_empty_texts:
            logger.warning("All texts in batch were empty after stripping.")
            # Return a list of empty lists matching the original batch size
            return [[] for _ in texts]

        try:
            # Use genai.embed_content for batch embeddings
            result = genai.embed_content(
                model=self.model,
                content=non_empty_texts,
                task_type="RETRIEVAL_DOCUMENT"
            )
            
            # Create a map of text to its embedding for correct ordering
            embeddings_map = {text: emb for text, emb in zip(non_empty_texts, result['embedding'])}
            
            # Reconstruct the list of embeddings, preserving order and empty slots
            final_embeddings = []
            for text in texts:
                if text.strip() in embeddings_map:
                    final_embeddings.append(embeddings_map[text.strip()])
                else:
                    final_embeddings.append([])
            
            logger.debug(f"Successfully generated embeddings for a batch of {len(non_empty_texts)} texts.")
            return final_embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings for batch. Error: {e}", exc_info=True)
            # Return a list of empty lists matching the original batch size on error
            return [[] for _ in texts]

# Example usage (for testing)
if __name__ == "__main__":
    from dotenv import load_dotenv
    load_dotenv()
    
    # The API key needs to be configured before using the embedder
    api_key = os.getenv("GOOGLE_API_KEY")
    if api_key:
        genai.configure(api_key=api_key)
        logger.info("Running Embedder example usage with Gemini.")
        
        embedder = Embedder()
        
        test_text = "This is a test sentence for Gemini embedding."
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
    else:
        logger.error("GOOGLE_API_KEY not found. Cannot run example.")