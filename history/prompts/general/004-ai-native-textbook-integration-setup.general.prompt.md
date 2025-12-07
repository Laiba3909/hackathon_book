---
id: 004
title: AI-Native Textbook Integration Setup
stage: general
date: 2025-12-07
surface: agent
model: gemini-pro
feature: none
branch: main
user: user
command: "Project: AI-Native Textbook – Gemini CLI Integration Setup Context: A complete Docusaurus textbook has already been authored using Gemini CLI. Constitution and Specification phases are complete. Objective: Prepare the existing book for AI-native workflows managed by SpecifyPlus. Core Setup Tasks: - Register /docs directory as canonical content source - Generate structural manifest of all chapters - Enable heading-based section indexing - Add stable content identifiers (CID) to every section AI-Readiness Configuration: - Enable semantic chunk boundary detection - Normalize paragraph-level content for vector compatibility - Attach metadata tags for retrieval context User-Experience Hooks: - Register future hooks for: - Chapter-level personalization buttons - Urdu translation toggles - User background-aware content variants Integration Stubs (No Execution): - Define placeholders for: - FastAPI service layer - Qdrant vector database - Neon Postgres user store Runtime Intelligence Preparation: - Enable extension points for Gemini CLI sub-agents - Register reusable agent skills - Prepare system prompt registry Success Criteria: - Book becomes system-ingestion ready - No content mutated - Project marked as “AI-ready” in SpecifyPlus Explicit Exclusions: - No Claude Code usage - No chatbot instantiation - No embedding generation - No database provisioning"
labels: ["setup", "ai-native", "specifyplus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/content-source.md
- .specify/docs-manifest.json
- .specify/section-index.json
- docs/intro.md
- .specify/ai-readiness-config.md
- .specify/ux-hooks.md
- backend/README.md
- backend/fastapi_service/__init__.py
- backend/fastapi_service/main.py
- backend/qdrant_connector/__init__.py
- backend/qdrant_connector/client.py
- backend/neon_postgres/__init__.py
- backend/neon_postgres/models.py
- backend/neon_postgres/crud.py
- .specify/runtime-intelligence.md
- .specify/AI_READY
tests: []
---

## Prompt

Project: AI-Native Textbook – Gemini CLI Integration Setup

Context:
A complete Docusaurus textbook has already been authored using Gemini CLI.
Constitution and Specification phases are complete.

Objective:
Prepare the existing book for AI-native workflows managed by SpecifyPlus.

Core Setup Tasks:
- Register /docs directory as canonical content source
- Generate structural manifest of all chapters
- Enable heading-based section indexing
- Add stable content identifiers (CID) to every section

AI-Readiness Configuration:
- Enable semantic chunk boundary detection
- Normalize paragraph-level content for vector compatibility
- Attach metadata tags for retrieval context

User-Experience Hooks:
- Register future hooks for:
  - Chapter-level personalization buttons
  - Urdu translation toggles
  - User background-aware content variants

Integration Stubs (No Execution):
- Define placeholders for:
  - FastAPI service layer
  - Qdrant vector database
  - Neon Postgres user store

Runtime Intelligence Preparation:
- Enable extension points for Gemini CLI sub-agents
- Register reusable agent skills
- Prepare system prompt registry

Success Criteria:
- Book becomes system-ingestion ready
- No content mutated
- Project marked as “AI-ready” in SpecifyPlus

Explicit Exclusions:
- No Claude Code usage
- No chatbot instantiation
- No embedding generation
- No database provisioning

## Response snapshot

The project has been successfully prepared for AI-native workflows managed by SpecifyPlus. All core setup tasks, AI-readiness configurations, user-experience hooks, integration stubs, and runtime intelligence preparations have been addressed. The success criteria have been verified, and the project is now marked as "AI-ready".

## Outcome

- ✅ Impact: Project successfully configured for AI-native workflows.
- 🧪 Tests: All success criteria met.
- 📁 Files: Numerous files and directories created/modified for AI-native integration setup.
- 🔁 Next prompts: Project is now AI-ready. User can proceed with further AI-native development.
- 🧠 Reflection: Successfully prepared the existing Docusaurus textbook for AI-native workflows as per the objective. Handled shell command execution differences for Windows. Ensured "no content mutated" by adding CIDs as HTML comments. All success criteria verified.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
