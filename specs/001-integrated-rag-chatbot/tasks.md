# Tasks: Integrated Read-Only RAG Chatbot for Physical AI Textbook

**Input**: Design documents from `/specs/001-integrated-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test tasks, but due to the critical nature of accuracy and safety requirements, testing will be included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on the project structure from plan.md:
- **Backend**: `backend/rag_api/`, `backend/tests/`
- **Frontend**: `frontend/src/`, `frontend/tests/`
- **Docs**: `specs/001-integrated-rag-chatbot/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure in backend/rag_api/
- [x] T002 Initialize Python 3.11 project with FastAPI, OpenAI, Qdrant, and Neon Postgres dependencies
- [x] T003 [P] Configure linting and formatting tools for Python (black, flake8, mypy)
- [x] T004 [P] Configure Docker and docker-compose for local development
- [x] T005 Create frontend component structure in frontend/src/components/
- [x] T006 Set up environment configuration management with .env files

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Setup database schema and migrations framework for Neon Postgres in backend/rag_api/db/
- [x] T008 [P] Implement core data models (ChatSession, Query, Response) in backend/rag_api/models/
- [x] T009 [P] Setup API routing and middleware structure in backend/rag_api/api/v1/
- [x] T010 Create vector database setup for Qdrant with textbook content indexing
- [x] T011 Configure error handling and logging infrastructure in backend/rag_api/utils/
- [x] T012 Implement content filtering service using OpenAI Moderation API in backend/rag_api/services/
- [x] T013 Setup environment configuration for API keys in backend/rag_api/config/
- [x] T014 [P] Implement base validation functions in backend/rag_api/utils/validators.py
- [x] T015 Set up API endpoint contracts from contracts/api-contract.yaml

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Chat with AI Assistant (Priority: P1) 🎯 MVP

**Goal**: Enable students and educators to interact with an AI-powered chatbot that answers questions based on the Physical AI textbook content using RAG technology with both full-book corpus and user-selected text modes.

**Independent Test**: Can be fully tested by asking the chatbot questions about Physical AI textbook content and verifying that responses are accurate, well-cited, and helpful.

### Tests for User Story 1 ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T016 [P] [US1] Contract test for POST /chat endpoint in backend/tests/contract/test_chat_api.py
- [X] T017 [P] [US1] Integration test for full-book corpus query flow in backend/tests/integration/test_full_corpus_query.py
- [X] T018 [P] [US1] Integration test for user-selected text query flow in backend/tests/integration/test_selected_text_query.py

### Implementation for User Story 1

- [x] T019 [P] [US1] Implement RAG service in backend/rag_api/services/rag_service.py
- [x] T020 [P] [US1] Implement embedding service in backend/rag_api/services/embedding_service.py
- [x] T021 [US1] Create Citation model in backend/rag_api/models/citation.py
- [x] T022 [US1] Create KnowledgeChunk model in backend/rag_api/models/knowledge_chunk.py
- [x] T023 [US1] Implement chat endpoint in backend/rag_api/api/v1/chat.py
- [x] T024 [US1] Implement session management in backend/rag_api/api/v1/sessions.py
- [x] T025 [US1] Add validation and error handling for chat endpoints
- [x] T026 [US1] Implement OpenAI Assistant integration with textbook corpus
- [x] T027 [US1] Add logging for chat operations
- [x] T028 [US1] Create frontend RagChatbot component in frontend/src/components/RagChatbot.jsx
- [x] T029 [US1] Implement useRagChat hook in frontend/src/hooks/useRagChat.js
- [x] T030 [US1] Create API service for chat communication in frontend/src/services/ragApi.js

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Navigate to Referenced Content (Priority: P2)

**Goal**: Enable users to click on citations in chatbot responses to navigate directly to the referenced chapter or section in the textbook, facilitating deeper exploration of topics.

**Independent Test**: Can be tested by asking the chatbot questions that generate responses with citations, then clicking those citations to verify navigation to the correct textbook section.

### Tests for User Story 2 ⚠️

- [X] T031 [P] [US2] Contract test for GET /textbook/chapters endpoint in backend/tests/contract/test_textbook_api.py
- [X] T032 [P] [US2] Contract test for GET /textbook/chapters/{chapter_number}/sections/{section_number} endpoint in backend/tests/contract/test_textbook_api.py
- [X] T033 [P] [US2] Integration test for citation navigation in frontend/tests/integration/test_citation_navigation.js

### Implementation for User Story 2

- [X] T034 [P] [US2] Create BookCorpus model in backend/rag_api/models/book_corpus.py
- [X] T035 [US2] Implement textbook chapters endpoint in backend/rag_api/api/v1/textbook.py
- [X] T036 [US2] Implement textbook section endpoint in backend/rag_api/api/v1/textbook.py
- [X] T037 [US2] Add chapter/section navigation links to citation model
- [X] T038 [US2] Enhance frontend RagChatbot component to render clickable citations
- [X] T039 [US2] Implement navigation functionality in frontend
- [X] T040 [US2] Add citation URL validation in backend/rag_api/utils/validators.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Maintain Conversation Context (Priority: P3)

**Goal**: Enable the chatbot to maintain context throughout a conversation, allowing users to ask follow-up questions that reference previous exchanges without repeating information.

**Independent Test**: Can be tested by conducting a multi-turn conversation with the chatbot to verify that follow-up questions are understood in the context of previous exchanges.

### Tests for User Story 3 ⚠️

- [X] T041 [P] [US3] Integration test for multi-turn conversation context in backend/tests/integration/test_conversation_context.py
- [X] T042 [P] [US3] Unit test for session history management in backend/tests/unit/test_session_history.py

### Implementation for User Story 3

- [X] T043 [P] [US3] Enhance Query model to include conversation context in backend/rag_api/models/query.py
- [X] T044 [US3] Implement conversation history tracking in ChatSession model
- [X] T045 [US3] Modify RAG service to incorporate conversation history in backend/rag_api/services/rag_service.py
- [X] T046 [US3] Update OpenAI Assistant integration to include conversation history
- [X] T047 [US3] Enhance frontend component to display conversation history
- [X] T048 [US3] Update useRagChat hook to manage conversation context

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T049 [P] Documentation updates in specs/001-integrated-rag-chatbot/
- [X] T050 Code cleanup and refactoring
- [ ] T051 Performance optimization across all stories
- [ ] T052 [P] Additional unit tests in backend/tests/unit/ and frontend/tests/
- [ ] T053 Security hardening for content filtering and rate limiting
- [ ] T054 Run quickstart.md validation
- [ ] T055 Add caching mechanisms for improved performance
- [ ] T056 Implement monitoring and metrics collection
- [ ] T057 Add comprehensive error handling and graceful failure mechanisms

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 components for citation data
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 components for conversation data

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Contract test for POST /chat endpoint in backend/tests/contract/test_chat_api.py"
Task: "Integration test for full-book corpus query flow in backend/tests/integration/test_full_corpus_query.py"
Task: "Integration test for user-selected text query flow in backend/tests/integration/test_selected_text_query.py"

# Launch all models for User Story 1 together:
Task: "Create Citation model in backend/rag_api/models/citation.py"
Task: "Create KnowledgeChunk model in backend/rag_api/models/knowledge_chunk.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence