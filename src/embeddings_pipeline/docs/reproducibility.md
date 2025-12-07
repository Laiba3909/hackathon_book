# Reproducibility in the Embeddings Pipeline

Ensuring reproducibility of the generated embeddings is critical for consistency, debugging, and validation of the AI-Native Textbook project. This document outlines the factors contributing to reproducibility and the steps taken to achieve it.

## Factors Affecting Reproducibility

Several factors can influence the determinism of embedding generation and storage:

1.  **Text Preprocessing/Normalization:** Variations in cleaning Markdown, handling whitespace, and special characters can alter the input text to the embedding model.
2.  **Chunking Strategy:** Different chunking logic (size, overlap, semantic splits) will produce different text segments, leading to different embeddings.
3.  **Embedding Model:** The specific embedding model used, its version, and the API provider's implementation can introduce non-determinism.
4.  **API Calls:** External API calls (e.g., to OpenAI) might not always return perfectly identical embeddings for the same input, especially if the API uses dynamic routing or model versions.
5.  **Qdrant Storage:** While Qdrant is deterministic in how it stores vectors, any issues in passing data or metadata can affect reproducibility during retrieval.

## Steps for Ensuring Reproducibility

The embeddings pipeline is designed with the following considerations to promote reproducibility:

### 1. Versioned Dependencies

All Python dependencies are specified in `requirements.txt` with exact versions. This ensures that the environment in which the pipeline runs is consistent.

### 2. Deterministic Text Processing

*   **Markdown Parsing:** The `MarkdownParser` is designed to consistently extract frontmatter, headings, and content.
*   **Text Cleaning:** The `TextCleaner` applies a fixed set of rules for stripping Markdown, standardizing whitespace, and handling special characters. Any changes to these rules will constitute a new version of the processing.
*   **Chunking Logic:** The `TextChunker` uses deterministic algorithms based on character/word counts and paragraph breaks to split content into chunks.

### 3. Stable Content Identifiers (CID)

Each section of the source Markdown documents is tagged with a Stable Content Identifier (CID). This CID provides a unique and persistent reference to the exact content section, allowing for tracking and verification across different runs.

### 4. Fixed Embedding Model

The pipeline explicitly uses a specific, versioned embedding model (e.g., `text-embedding-ada-002` from OpenAI). Using a fixed model reduces variability.

### 5. Environment Variable Management

All sensitive configurations (API keys, hosts) are managed via environment variables loaded from `.env`, ensuring that the pipeline runs with consistent external service configurations.

## Verification

To verify the reproducibility of the embeddings:

1.  Run the pipeline on the same set of input Markdown files multiple times.
2.  After each run, compare the generated embeddings (if possible to extract) and their associated metadata in Qdrant.
3.  Changes should only occur if the source content (Markdown files) or the pipeline's logic (parsing, cleaning, chunking, embedding model) has been intentionally updated.

By adhering to these principles, we aim to ensure that the process of generating vector embeddings from the textbook content is as deterministic and reproducible as possible.