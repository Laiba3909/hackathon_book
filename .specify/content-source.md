# Canonical Content Source Configuration for SpecifyPlus Workflows

## Designated Source Directory

The primary and canonical content source for all AI-native workflows managed by SpecifyPlus in this project is the `/docs` directory. This directory contains the Markdown source files for the "Physical AI & Humanoid Robotics Book," structured and managed by Docusaurus.

## Rationale and Implications

By designating `/docs` as the canonical source:

*   All content ingestion, processing, indexing, and transformation tasks will operate exclusively on the files found within `/docs`.
*   Changes or updates to the book's content for AI-native purposes (e.g., embedding generation, metadata tagging, CID integration) will originate from or be applied directly to these source Markdown files.
*   The Docusaurus build process will continue to render the book from this directory, ensuring consistency between the published book and the AI-ready content.
*   Any structural manifests, heading-based indexing, or content identifiers generated will be derived from and linked to this `/docs` content.

## Future SpecifyPlus Integration

Should SpecifyPlus introduce a formal configuration mechanism for content sources, this document will serve as the basis for that configuration. For now, adherence to this principle is ensured through all subsequent processing steps detailed in the AI-native workflow setup.