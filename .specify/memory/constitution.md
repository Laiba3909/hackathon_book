<!--
Sync Impact Report:
Version change: 0.1.0 -> 1.0.0
Modified principles: All principles replaced to focus on the RAG chatbot implementation.
Added sections: None
Removed sections: Academic Standards & Verification
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- README.md: ⚠ pending
Follow-up TODOs: None
-->
# Project Constitution: Gemini RAG Chatbot

## 1. Project Overview

### 1.1 Mission
Design and generate a production-ready RAG (Retrieval Augmented Generation) chatbot using the Gemini API.

### 1.2 Scope
#### In Scope
The chatbot must:
- Answer user queries only from provided documentation.
- Use vector search and retrieval before generating responses.
- Be suitable for technical documentation (Docusaurus / Specify commands).
- Ingest Markdown (.md, .mdx) files from a Docusaurus documentation.
- Logically chunk documents based on headings and sections.
- Store document embeddings in a vector store.

#### Out of Scope
- The chatbot will not use general internet knowledge.
- The chatbot will not answer questions outside the scope of the provided documentation.

## 2. Core Principles

### 2.1 Principle 1: Knowledge Source Integrity
**Description:** The chatbot MUST answer user queries only from the provided documentation. It is forbidden to use general internet knowledge or invent answers.
**Rationale:** To ensure all responses are accurate, verifiable, and grounded in the project's own source of truth.

### 2.2 Principle 2: Strict RAG Flow
**Description:** The chatbot MUST follow a strict Retrieval-Augmented Generation (RAG) flow. This means every user question is first converted into an embedding, used to retrieve relevant document chunks, and only then is the context injected into a Gemini prompt to generate an answer.
**Rationale:** This enforces the "Knowledge Source Integrity" principle and prevents the model from answering from its own knowledge.

### 2.3 Principle 3: Explicit "Not Found" Response
**Description:** If the answer to a user's query is not found in the retrieved document chunks, the chatbot MUST respond with: "Sorry, this information is not available in the documentation."
**Rationale:** To avoid hallucination and to clearly communicate the limits of the chatbot's knowledge base to the user.

### 2.4 Principle 4: Constrained and Technical Responses
**Description:** Answers MUST be clear, short, and technical. Where commands or code are involved, they MUST be presented in appropriate code blocks.
**Rationale:** To provide direct, useful, and easily readable answers for a technical audience.

### 2.5 Principle 5: Professional and Helpful Tone
**Description:** The chatbot's tone MUST be professional, helpful, and in the style of technical documentation. If a user's query is vague, the chatbot MUST ask a clarification question.
**Rationale:** To provide a good user experience and to elicit the necessary information to provide an accurate answer.

### 2.6 Principle 6: Modular and Well-Documented Code
**Description:** The implementation MUST follow a modular code structure (e.g., loader, embeddings, retriever, chat handler) and be clearly commented.
**Rationale:** To ensure the codebase is maintainable, extensible, and easy for other developers to understand.

### 2.7 Principle 7: Graceful Failure Handling
**Description:** If the documentation is empty or the retrieval process fails, the chatbot MUST return a graceful error message and MUST NOT attempt to guess an answer.
**Rationale:** To provide a robust user experience and prevent the generation of misleading information in case of system errors.

## 3. Technical Governance

### 3.1 Technical Governance Rules
- The Gemini API key MUST be loaded securely from a `.env` file and not be hardcoded in the source.
- The project will have a simple API or CLI interface for asking questions.
- The codebase will be structured into logical modules for different parts of the RAG pipeline.

### 3.2 Architecture Decision Standards
All architectural decisions MUST be:
- Documented, stating the decision, context, and rationale.
- Justified by their adherence to the Core Principles.
- Focused on creating a reliable and maintainable system.

## 4. Governance

### 4.1 Amendment Procedure
This Constitution may be amended by a unanimous consensus of the project lead team. Proposed amendments must be clearly documented, detailing the rationale and impact, and circulated to all stakeholders at least one week prior to deliberation.

### 4.2 Versioning Policy
This Constitution adheres to semantic versioning (MAJOR.MINOR.PATCH).
- MAJOR version increments indicate backward-incompatible changes, removal of principles, or fundamental shifts in governance.
- MINOR version increments indicate additions of new principles, sections, or materially expanded guidance.
- PATCH version increments indicate clarifications, wording adjustments, typo corrections, or non-semantic refinements.

### 4.3 Compliance Review Expectations
Compliance with this Constitution will be reviewed during project development. Any deviations or non-compliance issues MUST be documented and remediated.

---
**Governance Details**
- **Ratification Date:** 2025-12-07
- **Last Amended Date:** 2025-12-18
- **Constitution Version:** 1.0.0
