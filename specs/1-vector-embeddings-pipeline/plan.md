# Implementation Plan: Vector Embeddings Pipeline

**Feature:** Vector Embeddings Pipeline for AI-Native Textbook
**Version:** 1.0

## 1. Tech Stack

-   **Language:** Python 3.11+
-   **Vector Database:** Qdrant Cloud (Free Tier)
-   **Embedding Model:** `sentence-transformers/all-MiniLM-L6-v2` (or similar high-performance model from Hugging Face)
-   **HTTP Client:** `httpx` for interacting with Qdrant Cloud API
-   **Core Libraries:**
    -   `qdrant-client`: Official Python client for Qdrant.
    -   `sentence-transformers`: For generating embeddings.
    -   `numpy`: For numerical operations on vectors.
    -   `tqdm`: For progress bars during long-running processes.
    -   `python-dotenv`: For managing environment variables (API keys).
-   **Testing:** `pytest` for unit and integration tests.

## 2. Project Structure

The pipeline will be developed as a set of Python scripts and modules within the `src/embeddings_pipeline` directory.

```
src/embeddings_pipeline/
├── .env.example            # Example environment variables
├── main.py                 # Main script to run the full pipeline
├── pipeline.py             # Core pipeline logic (orchestrator)
├── embedder.py             # Handles text embedding generation
├── qdrant_client.py        # Manages connection and operations with Qdrant
├── utils/
│   ├── markdown_parser.py  # Parses and extracts text from markdown files
│   ├── text_cleaner.py     # Cleans and preprocesses text
│   └── chunking.py         # Implements the text chunking strategy
├── requirements.txt        # Python dependencies
└── README.md               # Instructions for setup and usage
tests/
└── embeddings_pipeline/
    ├── unit/
    │   ├── test_chunking.py
    │   ├── test_markdown_parser.py
    │   └── test_text_cleaner.py
    └── integration/
        ├── test_qdrant_connection.py
        └── test_full_pipeline.py
```

## 3. Key Decisions & Rationale

-   **Chunking Strategy:** We will implement a hierarchical chunking strategy. Documents will be split by headers first, then by paragraphs or sentences. This will be implemented in `utils/chunking.py`. This preserves the semantic context around related blocks of text.
-   **Embedding Model Choice:** `all-MiniLM-L6-v2` is chosen for its balance of performance and speed. It's a well-regarded model for semantic similarity tasks and is small enough to run efficiently.
-   **Deterministic Embeddings:** To ensure reproducibility, we will lock the versions of all libraries, especially `sentence-transformers`. The embedding process itself is deterministic given the same model and input.
-   **Qdrant Interaction:** All interactions with Qdrant will be encapsulated in the `qdrant_client.py` module. This will make it easy to manage the connection and to mock for testing purposes.
-   **Configuration:** API keys and other sensitive information will be managed via a `.env` file, which will not be checked into source control.

## 4. Implementation Phases

The implementation will follow the user stories outlined in `spec.md`.

-   **Phase 1: Setup & Foundational Components:**
    -   Initialize the project structure.
    -   Implement the utility functions for parsing, cleaning, and chunking.
    -   Implement the `QdrantClient` to connect to Qdrant Cloud.
-   **Phase 2: User Story 1 - Create and Store Embeddings:**
    -   Implement the `Embedder` to generate vector embeddings.
    -   Implement the main `pipeline.py` to orchestrate the process of reading, chunking, embedding, and storing the documents.
    -   Create the `main.py` script to run the pipeline from the command line.
-   **Phase 3: Testing:**
    -   Write unit tests for all utility functions.
    -   Write integration tests for the Qdrant connection and the full pipeline.

## 5. Testing Strategy

-   **Unit Tests:** Each utility function (`markdown_parser`, `text_cleaner`, `chunking`) will have comprehensive unit tests to ensure correctness.
-   **Integration Tests:**
    -   A test will verify the connection to the Qdrant Cloud instance and basic operations (create collection, upsert points).
    -   A full end-to-end pipeline test will process a small sample of markdown files, generate embeddings, store them in a test collection in Qdrant, and then perform a similarity search to verify the results.
-   **Manual Testing:** The final pipeline will be run on the full textbook, and the results will be manually inspected to ensure they meet the quality criteria.
