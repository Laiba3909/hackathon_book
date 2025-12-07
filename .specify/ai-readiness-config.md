# AI-Readiness Configuration for AI-Native Textbook

This document outlines the strategies for preparing the textbook content for AI-native workflows, specifically focusing on semantic chunking, content normalization, and metadata tagging.

## 1. Semantic Chunk Boundary Detection

The initial coarse-grained chunking will leverage the existing structural elements of the Docusaurus markdown files, specifically:

*   **Document Boundaries:** Each `.md` file represents a distinct document or sub-document, serving as a primary chunking unit.
*   **Heading-Based Sectioning:** Headings (H1, H2, H3, etc.) within each Markdown file will define logical section boundaries. The content immediately following a heading, up to the next heading of an equal or higher level, constitutes a semantic segment.
*   **Stable Content Identifiers (CIDs):** Each heading will be preceded by a unique, stable CID (e.g., `<!-- CID: <hash> -->`). These CIDs will serve as persistent identifiers for these chunks, enabling reproducible referencing and retrieval.

Further, more granular chunking (e.g., paragraph-level) will occur *within* these heading-defined sections, ensuring that semantic context is preserved. Boundary detection at the paragraph level will consider natural breaks in the text.

## 2. Content Normalization for Vector Compatibility

To ensure optimal performance and compatibility with vector embedding models, the extracted text content within each chunk will undergo normalization:

*   **Markdown Stripping:** All Markdown syntax (e.g., `**bold**`, `*italic*`, `[link](url)`, code blocks, image links) will be removed, retaining only the plain text.
*   **Whitespace Standardization:** Multiple spaces, tabs, and newline characters will be collapsed into single spaces to reduce noise.
*   **Special Character Handling:** Non-standard characters or emojis will be removed or replaced.
*   **Lowercase Conversion (Optional/Configurable):** The entire text can be converted to lowercase, depending on the requirements of the chosen embedding model and retrieval strategy. This can be made configurable.
*   **Code Block Identification:** Code blocks will be identified and potentially treated differently (e.g., excluded from general text embedding, or embedded separately) to avoid polluting general semantic understanding.
*   **Frontmatter Removal:** Docusaurus frontmatter (YAML block at the top of the Markdown files) will be removed before normalization.

The goal is to produce a clean, consistent, and semantically rich text string for each chunk, maximizing its utility for vectorization.

## 3. Metadata Tagging for Retrieval Context

Each semantic chunk (defined by heading and CID) will be enriched with a comprehensive set of metadata tags. This metadata is critical for:

*   **Contextual Retrieval:** Allowing the RAG system to retrieve not just the relevant text, but also its origin and context within the textbook.
*   **Filtering and Faceting:** Enabling advanced search and retrieval operations based on structural or topical criteria.

The metadata schema for each chunk will include, but not be limited to:

*   **`doc_id`**: The Docusaurus document ID (e.g., `intro`, `ros-2-nervous-system/introduction`).
*   **`filepath`**: The relative path to the source Markdown file (e.g., `docs/intro.md`).
*   **`chapter_label`**: The label of the chapter the chunk belongs to (e.g., "Chapter 1: The Robotic Nervous System"), derived from `sidebars.ts`.
*   **`section_title`**: The title of the heading under which the chunk falls.
*   **`cid`**: The Stable Content Identifier generated for that specific section.
*   **`heading_level`**: The Markdown heading level (e.g., 1 for H1, 2 for H2).
*   **`sidebar_position`**: (If available from frontmatter) The position in the sidebar.
*   **`original_title_frontmatter`**: (If available from frontmatter) The title defined in the Markdown frontmatter.
*   **`page_title`**: The main title of the document as presented in Docusaurus.

This robust metadata framework ensures that each extracted text chunk is fully contextualized, greatly enhancing the precision and relevance of retrieval in the RAG system.