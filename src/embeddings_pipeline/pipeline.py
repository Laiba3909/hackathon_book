# src/embeddings_pipeline/pipeline.py

import os
import json
import logging
from typing import List, Dict
from tenacity import retry, stop_after_attempt, wait_fixed, retry_if_exception_type

from src.embeddings_pipeline.utils.markdown_parser import MarkdownParser
from src.embeddings_pipeline.utils.chunking import TextChunker
from src.embeddings_pipeline.utils.text_cleaner import TextCleaner
from src.embeddings_pipeline.embedder import Embedder
from src.embeddings_pipeline.qdrant_client import QdrantClientWrapper
from src.embeddings_pipeline.logger import setup_logging # Import the setup_logging function

logger = setup_logging(__name__)

class EmbeddingsPipeline:
    def __init__(self, docs_base_path: str = '../../docs', collection_name: str = 'ai_native_textbook_embeddings'):
        self.docs_base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), docs_base_path))
        self.collection_name = collection_name
        
        self.markdown_parser = MarkdownParser()
        self.text_cleaner = TextCleaner()
        self.text_chunker = TextChunker(max_chunk_size=200, overlap=50) # Configure chunk size/overlap
        self.embedder = Embedder()
        self.qdrant_wrapper = QdrantClientWrapper()

        # Load docs manifest (conceptual for now, used to get list of files)
        self.docs_manifest_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../.specify/docs-manifest.json'))
        try:
            with open(self.docs_manifest_path, 'r', encoding='utf-8') as f:
                self.docs_manifest = json.load(f)
            logger.info(f"Loaded docs manifest from {self.docs_manifest_path}")
        except FileNotFoundError:
            logger.error(f"Docs manifest not found at {self.docs_manifest_path}. Pipeline cannot proceed.")
            self.docs_manifest = []

    def _get_all_markdown_filepaths(self):
        """
        Recursively extracts all markdown file paths from the docs manifest.
        """
        filepaths = []
        def traverse(items):
            for item in items:
                if item['type'] == 'document':
                    filepaths.append(item['filepath'])
                elif item['type'] == 'category':
                    if item.get('introduction'):
                        filepaths.append(item['introduction']['filepath'])
                    traverse(item.get('items', []))
        traverse(self.docs_manifest)
        return filepaths

    @retry(stop=stop_after_attempt(3), wait=wait_fixed(2), reraise=True)
    def _get_embeddings_with_retry(self, texts: List[str]) -> List[List[float]]:
        """Retry mechanism for embedding generation."""
        return self.embedder.get_embeddings_batch(texts)

    def run_pipeline(self):
        logger.info(f"Starting embeddings pipeline for collection: {self.collection_name}")
        
        try:
            health = self.qdrant_wrapper.health_check()
            if health["status"] == "error":
                logger.error(f"Qdrant health check failed: {health['message']}. Aborting pipeline.")
                return

            if not self.qdrant_wrapper.get_or_create_collection(self.collection_name, self.embedder.vector_size):
                logger.error(f"Failed to ensure Qdrant collection '{self.collection_name}' exists or was created. Aborting pipeline.")
                return

            markdown_filepaths = self._get_all_markdown_filepaths()
            if not markdown_filepaths:
                logger.warning("No Markdown files found in the manifest. Pipeline has nothing to process.")
                return

            for relative_filepath in markdown_filepaths:
                full_filepath = os.path.join(self.docs_base_path, relative_filepath.replace('docs/', '')) # Adjust pathing
                
                logger.info(f"Processing {relative_filepath}...")
                frontmatter, sections = self.markdown_parser.parse_markdown_file(full_filepath)

                if not sections:
                    logger.warning(f"No sections found in {relative_filepath}, skipping.")
                    continue

                doc_id = relative_filepath.replace('docs/', '').replace('.md', '')
                
                for section in sections:
                    section['doc_id'] = doc_id
                    section['filepath'] = relative_filepath
                    # Further metadata enrichment from manifest can go here

                cleaned_chunks_for_doc = []
                for section in sections:
                    cleaned_content = self.text_cleaner.clean_text(
                        section['content'], 
                        strip_markdown=True, 
                        remove_special_chars=True, 
                        to_lowercase=True
                    )
                    
                    raw_chunks = self.text_chunker.chunk_section_content(cleaned_content)
                    
                    for i, chunk_text in enumerate(raw_chunks):
                        chunk_metadata = {
                            "doc_id": section['doc_id'],
                            "filepath": section['filepath'],
                            "section_title": section['title'],
                            "cid": section['cid'],
                            "heading_level": section['level'],
                            "chunk_index": i,
                            "total_chunks_in_section": len(raw_chunks)
                        }
                        cleaned_chunks_for_doc.append({"text": chunk_text, "metadata": chunk_metadata})
                
                texts_to_embed = [chunk['text'] for chunk in cleaned_chunks_for_doc]
                
                try:
                    embeddings = self._get_embeddings_with_retry(texts_to_embed)
                except Exception as e:
                    logger.error(f"Failed to generate embeddings for {relative_filepath} after multiple retries: {e}")
                    continue # Skip to next file if embedding fails

                vectors_to_upsert = []
                payloads_to_upsert = []
                
                for i, chunk in enumerate(cleaned_chunks_for_doc):
                    if embeddings[i] and len(embeddings[i]) == self.embedder.vector_size: # Ensure valid embedding
                        vectors_to_upsert.append(embeddings[i])
                        payloads_to_upsert.append(chunk['metadata'])
                    else:
                        logger.warning(f"Skipping chunk {i} from {relative_filepath} due to missing or invalid embedding.")

                if vectors_to_upsert:
                    if self.qdrant_wrapper.upsert_vectors(self.collection_name, vectors_to_upsert, payloads_to_upsert):
                        logger.info(f"Successfully upserted {len(vectors_to_upsert)} embeddings for {relative_filepath}.")
                    else:
                        logger.error(f"Failed to upsert embeddings for {relative_filepath}.")
                else:
                    logger.info(f"No valid embeddings to upsert for {relative_filepath}.")

            logger.info("Embeddings pipeline finished successfully.")

        except Exception as e:
            logger.critical(f"An unhandled error occurred during pipeline execution: {e}", exc_info=True)

if __name__ == "__main__":
    # Ensure .env is configured with QDRANT_HOST, QDRANT_API_KEY, OPENAI_API_KEY
    pipeline = EmbeddingsPipeline()
    pipeline.run_pipeline()
