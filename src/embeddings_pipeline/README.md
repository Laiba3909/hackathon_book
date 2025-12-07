# Embeddings Pipeline for AI-Native Textbook

This directory contains the Python-based pipeline responsible for converting the Docusaurus textbook content into vector embeddings and storing them in Qdrant Cloud for Retrieval-Augmented Generation (RAG) purposes.

## Features

-   **Markdown Parsing:** Reads Docusaurus Markdown files, extracts frontmatter, headings, and content.
-   **Content Chunking:** Breaks down document sections into semantically coherent smaller chunks, preserving context.
-   **Text Normalization:** Cleans text by stripping Markdown, standardizing whitespace, and removing special characters, preparing it for embedding.
-   **Embedding Generation:** Utilizes OpenAI's `text-embedding-ada-002` model to generate high-dimensional vector representations of text chunks.
-   **Qdrant Integration:** Connects to Qdrant Cloud to create collections, store embeddings with rich metadata, and perform similarity searches.
-   **Robustness:** Includes logging, error handling, and retry mechanisms for API calls.

## Setup and Configuration

### 1. Clone the Repository

If you haven't already, clone this project repository:

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to the Pipeline Directory

```bash
cd src/embeddings_pipeline
```

### 3. Virtual Environment

Create and activate a Python virtual environment to manage dependencies:

```bash
python -m venv .venv
# On Windows:
.venv\Scripts\activate
# On macOS/Linux:
source .venv/bin/activate
```

### 4. Install Dependencies

Install the required Python packages:

```bash
pip install -r requirements.txt
```

### 5. Environment Variables

Create a `.env` file in the `src/embeddings_pipeline/` directory (or in the project root if preferred, ensuring proper `.env` loading) based on the provided `.env.example`:

```ini
# src/embeddings_pipeline/.env
QDRANT_HOST="https://YOUR_QDRANT_HOST_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
OPENAI_API_KEY="YOUR_OPENAI_API_KEY"
```

*   **`QDRANT_HOST`**: The URL for your Qdrant Cloud instance.
*   **`QDRANT_API_KEY`**: Your API key for authenticating with Qdrant Cloud.
*   **`OPENAI_API_KEY`**: Your API key for OpenAI, used for generating text embeddings.

Ensure these variables are correctly set before running the pipeline.

## Running the Pipeline

To execute the embeddings pipeline, navigate to the `src/embeddings_pipeline` directory and run the `main.py` script:

```bash
# Ensure your virtual environment is activated
python main.py
```

This will:
1.  Perform a health check on the Qdrant instance.
2.  Ensure the target Qdrant collection exists (or create it if it doesn't).
3.  Process all Markdown files found in the `docs/` directory (as defined by the Docusaurus `sidebars.ts` manifest).
4.  Parse, clean, chunk, and embed the content.
5.  Upsert the generated embeddings along with their metadata to Qdrant Cloud.
6.  Log the progress and any errors to both the console and a timestamped log file in the `logs/` directory.

## Testing

To run the unit and integration tests for the pipeline:

```bash
# Ensure your virtual environment is activated
# Navigate to the root of the project
python -m unittest discover tests/embeddings_pipeline
```

Refer to the `tests/embeddings_pipeline/` directory for specific test files.

## Documentation

-   **Reproducibility:** See `src/embeddings_pipeline/docs/reproducibility.md` for details on ensuring deterministic embedding generation.

## Future Enhancements

-   Implement a more dynamic chunking strategy.
-   Support for different embedding models (e.g., HuggingFace models).
-   Command-line arguments for collection name, chunking parameters, etc.
-   API endpoint exposing search functionality.
