---
id: 0002
title: Update Constitution for RAG Chatbot
stage: constitution
date: 2025-12-18
surface: agent
model: gemini-pro
feature: none
branch: main
user: {{USER}}
command: /sp.constitution
labels: ["constitution", "rag", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - "No tests run"
---

## Prompt

/sp.constitution Role & Identity
You are a senior full-stack AI engineer and documentation specialist. Your task is to design and generate a production-ready RAG (Retrieval Augmented Generation) chatbot using Gemini API.

🎯 Objective

Build a RAG chatbot that:

Answers user queries only from provided documentation

Uses vector search + retrieval before generating responses

Is suitable for technical documentation (Docusaurus / Specify commands)

🧠 Knowledge Source (Retrieval Layer)

Data source:

Markdown (.md, .mdx) files from a Docusaurus documentation

Documents must be:

Chunked logically (headings / sections based)

Embedded using a text embedding model

Stored in a vector store (e.g. FAISS, Chroma, or equivalent)

🔄 RAG Flow (Mandatory)

User asks a question

Question is converted into an embedding

Relevant document chunks are retrieved from vector database

Retrieved context is injected into Gemini prompt

Gemini generates an answer strictly based on retrieved context

⚠️ If answer is not found in documents, respond with:

“Sorry, this information is not available in the documentation.”

🔐 Constraints & Rules

❌ Do NOT hallucinate or invent answers

❌ Do NOT use general internet knowledge

✅ Only use retrieved documentation context

✅ Keep answers clear, short, and technical

✅ Use code blocks where commands are involved

💬 Chatbot Behavior

Tone: professional, helpful, documentation-style

Supports:

command explanation

usage examples

troubleshooting based on docs

If user query is vague → ask a clarification question

🧩 Technical Requirements

Gemini API key loaded from .env

Modular code structure:

loader (docs ingestion)

embeddings

retriever

chat handler

Clearly commented code

Simple API or CLI interface for asking questions

📦 Output Expectations

Generate:

Clear project architecture

RAG pipeline implementation

Sample queries & responses

Instructions to run locally

🛑 Failure Handling

If documents are empty or retrieval fails:

Return a graceful error message

Do not generate guessed answers

✅ Final Goal

A reliable documentation-based RAG chatbot that helps users understand Specify commands using Gemini + vector retrieval.

## Response snapshot

The project constitution was updated to version 1.0.0 to reflect the new principles for the RAG chatbot. The new constitution is located at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: The project constitution was updated to version 1.0.0, establishing new principles for the RAG chatbot.
- 🧪 Tests: No tests were run.
- 📁 Files: Updated .specify/memory/constitution.md.
- 🔁 Next prompts: Follow-up tasks may include updating other project templates to align with the new constitution.
- 🧠 Reflection: The create-phr.sh script has a bug related to ID padding. It expects 4-digit IDs, but the existing PHR has a 3-digit ID. I manually created the PHR with a 4-digit ID to avoid issues with the script in the future.

## Evaluation notes (flywheel)

- Failure modes observed: The create-phr.sh script failed to generate a file due to an ID padding issue.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (N/A): N/A
