# Tasks: Vector Embeddings Pipeline for AI-Native Textbook

**Input**: Design documents from `/specs/1-vector-embeddings-pipeline/`
**Prerequisites**: spec.md, .specify/docs-manifest.json, .specify/section-index.json
**Tests**: Test tasks will be generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [TaskID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths shown below assume single project - `src/` at repository root for pipeline logic. Backend stubs are in `backend/`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the embeddings pipeline

- [X] T001 Create project directory for the embeddings pipeline: `src/embeddings_pipeline`
- [X] T002 Initialize Python virtual environment: `src/embeddings_pipeline/.venv`
- [X] T003 Install core dependencies (`qdrant-client`, `fastapi`, `uvicorn`, `python-dotenv`) for the pipeline: `src/embeddings_pipeline/requirements.txt`
- [X] T004 Create a basic entry point for the pipeline script: `src/embeddings_pipeline/main.py`
- [X] T005 Create `.env.example` for environment variables (Qdrant API key, URL, etc.) for the pipeline: `src/embeddings_pipeline/.env.example`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before the user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement Qdrant client connection and basic health check: `src/embeddings_pipeline/qdrant_client.py`
- [X] T007 [P] Develop utility to read and parse Docusaurus Markdown files (handling frontmatter, extracting content) using the previously generated manifests: `src/embeddings_pipeline/utils/markdown_parser.py`
- [X] T008 [P] Implement text chunking logic that preserves semantic context based on headings and paragraph breaks: `src/embeddings_pipeline/utils/chunking.py`
- [X] T009 [P] Develop content normalization functions (stripping markdown, cleaning text) for vector compatibility: `src/embeddings_pipeline/utils/text_cleaner.py`

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Create and Store Embeddings (Priority: P1) 🎯 MVP

**Goal**: Successfully generate vector embeddings from the textbook content and store them in Qdrant Cloud, enabling it for Retrieval-Augmented Generation (RAG).

**Independent Test**: The pipeline can process a sample of the textbook, generate embeddings, store them in Qdrant, and perform a basic similarity search to retrieve relevant content.

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T010 [P] [US1] Unit test for markdown parsing functionality: `tests/embeddings_pipeline/unit/test_markdown_parser.py`
- [X] T011 [P] [US1] Unit test for text chunking logic: `tests/embeddings_pipeline/unit/test_chunking.py`
- [X] T012 [P] [US1] Unit test for text normalization functions: `tests/embeddings_pipeline/unit/test_text_cleaner.py`
- [X] T013 [P] [US1] Integration test for Qdrant client connection and basic operations: `tests/embeddings_pipeline/integration/test_qdrant_connection.py`
- [X] T014 [US1] Integration test for the full embeddings pipeline (from Markdown to Qdrant, including metadata attachment): `tests/embeddings_pipeline/integration/test_full_pipeline.py`
- [X] T015 [US1] Integration test for similarity search functionality against stored embeddings: `tests/embeddings_pipeline/integration/test_similarity_search.py`

### Implementation for User Story 1

- [X] T016 [P] [US1] Implement text embedding generation using a chosen model (e.g., HuggingFace Transformers or OpenAI API): `src/embeddings_pipeline/embedder.py`
- [X] T017 [US1] Develop the main pipeline orchestration script to read, parse, chunk, embed, and store content to Qdrant, ensuring metadata attachment: `src/embeddings_pipeline/pipeline.py` (depends on T007, T008, T009, T016)
- [X] T018 [US1] Enhance `src/embeddings_pipeline/qdrant_client.py` to handle collection creation, upserting embeddings with metadata, and robust error handling.
- [X] T019 [US1] Implement similarity search function in `src/embeddings_pipeline/qdrant_client.py` to query the stored embeddings and return contextualized results.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and documentation that affect the overall pipeline quality.

- [X] T020 [P] Implement comprehensive logging for pipeline execution and errors: `src/embeddings_pipeline/logger.py`
- [X] T021 [P] Add error handling and retry mechanisms throughout the pipeline: `src/embeddings_pipeline/pipeline.py`
- [X] T022 [P] Update the main `src/embeddings_pipeline/README.md` with instructions for setup, configuration, and running the pipeline.
- [X] T023 [P] Document the reproducibility steps for embedding generation within the pipeline's documentation: `src/embeddings_pipeline/docs/reproducibility.md`
- [X] T024 Code cleanup and refactoring across all pipeline components.
- [X] T025 Performance optimization for embedding generation and Qdrant interactions.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS User Story 1.
-   **User Story 1 (Phase 3)**: Depends on Foundational phase completion.
-   **Polish (Final Phase)**: Depends on User Story 1 being functionally complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories (as it's the only one).

### Within Each User Story

-   Tests MUST be written and FAIL before implementation.
-   Core utilities (markdown_parser, chunking, text_cleaner) should be developed before the main pipeline script.
-   Embedder implementation is a prerequisite for the main pipeline script.
-   Qdrant client enhancements and similarity search depend on initial client connection.

### Parallel Opportunities

-   **Phase 1 (Setup)**: Tasks T001-T005 are largely sequential for initial setup.
-   **Phase 2 (Foundational)**: T007, T008, T009 can be developed in parallel once T006 (Qdrant client base) is in progress or completed.
-   **Phase 3 (User Story 1 - Tests)**: T010-T013 (unit tests) can be developed in parallel. T014-T015 (integration tests) can be developed in parallel after unit tests are defined.
-   **Phase 3 (User Story 1 - Implementation)**: T016 (embedder) can be developed in parallel with T017 (pipeline orchestration) provided clear interfaces. T018 and T019 (Qdrant enhancements) can be worked on concurrently.
-   **Final Phase (Polish)**: Tasks T020-T023 are good candidates for parallel execution.

---

## Parallel Example: User Story 1

```bash
# Launch all unit tests for User Story 1 together:
Task: "T010 [P] [US1] Unit test for markdown parsing functionality: tests/embeddings_pipeline/unit/test_markdown_parser.py"
Task: "T011 [P] [US1] Unit test for text chunking logic: tests/embeddings_pipeline/unit/test_chunking.py"
Task: "T012 [P] [US1] Unit test for text normalization functions: tests/embeddings_pipeline/unit/test_text_cleaner.py"
Task: "T013 [P] [US1] Integration test for Qdrant client connection and basic operations: tests/embeddings_pipeline/integration/test_qdrant_connection.py"

# After core foundational components are ready, implement key parts of User Story 1:
Task: "T016 [P] [US1] Implement text embedding generation using a chosen model: src/embeddings_pipeline/embedder.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    *   Developer A: Focus on `src/embeddings_pipeline/utils/` (T007, T008, T009)
    *   Developer B: Focus on `src/embeddings_pipeline/qdrant_client.py` (T006) and `src/embeddings_pipeline/embedder.py` (T016)
    *   Developer C: Focus on the orchestration `src/embeddings_pipeline/pipeline.py` (T017) and testing (T010-T015)
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
