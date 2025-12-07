---
id: 001
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-pro
feature: none
branch: main
user: user
command: "/sp.constitution Create a Speckit Constitution for the project: “Integrated RAG Chatbot Development for an AI-Native Textbook”. PROJECT SCOPE: Design, implement, and document a Retrieval-Augmented Generation (RAG) chatbot embedded within a published technical textbook. The chatbot must: - Answer user questions about the textbook content - Support answering based only on user-selected text - Use OpenAI Agents/ChatKit SDKs - Use FastAPI as the backend framework - Use Qdrant Cloud Free Tier as the vector database - Be production-ready and academically documented CORE PRINCIPLES: 1. Accuracy through primary source execution - All technical explanations must prioritize original research papers, specifications, and official documentation. 2. Clarity for academic audiences (computer science background) - Writing must be technically precise yet readable for university-level CS students. 3. Reproducibility - All claims must be traceable, testable, and repeatable. - Include setup steps, configuration files, and reproducible experiment procedures. 4. Rigor - Prefer peer-reviewed sources over blogs or marketing material. - Include critical analysis and limitations. KEY STANDARDS: - All factual claims must be traceable to sources. - Citation format: APA Style - Minimum 50% of sources must be peer-reviewed articles - Plagiarism check: 0% tolerance before submission - Writing clarity must target Flesch–Kincaid grade 10–12 CONSTRAINTS: - Total word count: 5,000–7,000 words - Minimum 15 sources - Output format: PDF-ready academic document with embedded APA citations SUCCESS CRITERIA: - All technical and factual claims verified against sources - Zero plagiarism detected - Passes independent fact-checking review REQUIRED OUTPUT: Generate a formal “Speckit Constitution” that will govern how this RAG Chatbot system is implemented, documented, tested, and validated. The constitution must include: - Technical governance rules - Architecture decision standards - Source verification workflow - Citation and validation pipeline - Quality gates for academic publication"
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests: []
---

## Prompt

/sp.constitution Create a Speckit Constitution for the project: 
“Integrated RAG Chatbot Development for an AI-Native Textbook”.

PROJECT SCOPE:
Design, implement, and document a Retrieval-Augmented Generation (RAG) chatbot embedded within a published technical textbook. 
The chatbot must:
- Answer user questions about the textbook content
- Support answering based only on user-selected text
- Use OpenAI Agents/ChatKit SDKs
- Use FastAPI as the backend framework
- Use Qdrant Cloud Free Tier as the vector database
- Be production-ready and academically documented

CORE PRINCIPLES:
1. Accuracy through primary source execution
   - All technical explanations must prioritize original research papers, specifications, and official documentation.

2. Clarity for academic audiences (computer science background)
   - Writing must be technically precise yet readable for university-level CS students.

3. Reproducibility
   - All claims must be traceable, testable, and repeatable.
   - Include setup steps, configuration files, and reproducible experiment procedures.

4. Rigor
   - Prefer peer-reviewed sources over blogs or marketing material.
   - Include critical analysis and limitations.

KEY STANDARDS:
- All factual claims must be traceable to sources.
- Citation format: APA Style
- Minimum 50% of sources must be peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity must target Flesch–Kincaid grade 10–12

CONSTRAINTS:
- Total word count: 5,000–7,000 words
- Minimum 15 sources
- Output format: PDF-ready academic document with embedded APA citations

SUCCESS CRITERIA:
- All technical and factual claims verified against sources
- Zero plagiarism detected
- Passes independent fact-checking review

REQUIRED OUTPUT:
Generate a formal “Speckit Constitution” that will govern how this RAG Chatbot system is implemented, documented, tested, and validated.
The constitution must include:
- Technical governance rules
- Architecture decision standards
- Source verification workflow
- Citation and validation pipeline
- Quality gates for academic publication

## Response snapshot

The project constitution `v0.1.0` has been created and saved to `.specify/memory/constitution.md`. This is the initial version of the constitution, establishing the core principles, technical governance, academic standards, and project governance for "Integrated RAG Chatbot Development for an AI-Native Textbook."

**Manual Follow-up Required:**
The following files and areas require manual review and potential updates to align with the new constitution:
*   `.specify/templates/plan-template.md`
*   `.specify/templates/spec-template.md`
*   `.specify/templates/tasks-template.md`
*   All command files in `.specify/templates/commands/` (e.g., `sp.adr.toml`, `sp.plan.toml`, etc.)
*   `README.md`
*   `docs/quickstart.md` (if it exists)
*   Consider defining explicit "Out of Scope" items for the project.

**Suggested Commit Message:**
```
docs: add initial project constitution v0.1.0
```

## Outcome

- ✅ Impact: Initial project constitution drafted and saved.
- 🧪 Tests: No explicit tests were run for constitution creation.
- 📁 Files: Created .specify/memory/constitution.md.
- 🔁 Next prompts: Follow-up on updating related templates and defining out-of-scope items.
- 🧠 Reflection: Successfully created the initial project constitution based on user-provided requirements. Ensured all sections were populated and versioning/dates were set correctly. Identified dependent files for future updates.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
