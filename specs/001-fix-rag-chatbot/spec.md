# Feature Specification: Fix RAG Chatbot with Bonsai Qwen Integration

**Feature Branch**: `001-fix-rag-chatbot`
**Created**: 2025-12-28
**Status**: Draft
**Input**: User description: "Fix RAG chatbot with Bonsai Qwen integration - silent chatbot, low indexing points, missing Login/Signup UI"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Ingestion and Knowledge Base Setup (Priority: P1)

As a system administrator, I need to upload and ingest books into the knowledge base so that users can receive accurate answers based on the book content.

**Why this priority**: This is the foundation - without properly ingested and indexed content, the chatbot cannot answer questions. The current issue of only ~5 indexing points indicates ingestion is broken or incomplete.

**Independent Test**: Can be fully tested by uploading a test book and verifying it is chunked, embedded, and stored in the vector database. Delivers a functional knowledge base ready for querying.

**Acceptance Scenarios**:

1. **Given** a book file (PDF, TXT, or EPUB) is uploaded, **When** the ingestion process completes, **Then** the book is split into semantically meaningful chunks and each chunk is stored in the knowledge base
2. **Given** a large book (100+ pages), **When** ingestion completes, **Then** all content is processed without truncation and chunking preserves context boundaries
3. **Given** ingestion fails mid-process, **When** the error occurs, **Then** partial data is cleaned up and an error message is logged for the administrator

---

### User Story 2 - Question Answering with Retrieval (Priority: P1)

As a user, I need to ask questions about the book content and receive accurate answers based on the ingested material.

**Why this priority**: This is the core value proposition. The chatbot currently provides no answers (silent), which is the critical failure blocking all functionality.

**Independent Test**: Can be fully tested by asking questions about known book content and verifying answers are generated from retrieved chunks. Delivers a working question-answering experience.

**Acceptance Scenarios**:

1. **Given** the knowledge base contains ingested content, **When** a user submits a question, **Then** the system retrieves relevant chunks and generates an answer within 10 seconds
2. **Given** the question has relevant content in the knowledge base, **When** the answer is generated, **Then** the answer cites or references the specific book sections used
3. **Given** no relevant content exists, **When** a user asks a question, **Then** the system responds with a helpful message indicating no relevant information was found
4. **Given** a question about a specific book section, **When** using selected-text mode, **Then** only the highlighted text is used for retrieval and answer generation

---

### User Story 3 - Authentication UI Access (Priority: P1)

As a user, I need to see and use Login and Signup interfaces to access the chatbot.

**Why this priority**: The authentication UI is completely invisible, blocking all user access. Without authentication, users cannot interact with the system.

**Independent Test**: Can be fully tested by navigating to the application and verifying Login and Signup forms are visible and functional. Delivers user access to the chatbot.

**Acceptance Scenarios**:

1. **Given** a new user visits the application, **When** they navigate to the login page, **Then** they see a visible login form with username/email and password fields
2. **Given** a new user visits the application, **When** they navigate to the signup page, **Then** they see a visible signup form with required fields and a submit button
3. **Given** the user enters invalid credentials, **When** they submit the login form, **Then** an error message is displayed and the form remains visible
4. **Given** the user successfully signs up, **When** the process completes, **Then** they are redirected to the chatbot interface and logged in

---

### User Story 4 - Vector Indexing Quality (Priority: P2)

As a system administrator, I need the vector database to contain sufficient indexing points to ensure effective retrieval across the entire book content.

**Why this priority**: Current indexing shows only ~5 points, which indicates severe data loss during ingestion. This blocks effective retrieval even if the rest of the pipeline works.

**Independent Test**: Can be fully tested by ingesting a medium-sized book and verifying the number of indexed chunks correlates with expected chapter/section count. Delivers effective search coverage.

**Acceptance Scenarios**:

1. **Given** a book with 20 chapters, **When** ingestion completes, **Then** the vector database contains at least 20 indexed chunks (one per chapter minimum, more for large chapters)
2. **Given** a book is ingested, **When** querying for content from different chapters, **Then** results are returned from multiple chapters across the book
3. **Given** embedding creation fails, **When** the error occurs, **Then** the system retries up to 3 times before logging the failure and marking the chunk as unindexed

---

### User Story 5 - Quota-Efficient Operations (Priority: P2)

As a system administrator operating on free-tier quotas, I need the system to minimize API calls to Bonsai/Qwen and other external services.

**Why this priority**: Free-tier limits are restrictive. Excessive API calls will quickly exhaust quotas and stop the system from working.

**Independent Test**: Can be fully tested by monitoring API call counts during ingestion and Q&A sessions. Delivers sustainable operation within free limits.

**Acceptance Scenarios**:

1. **Given** a book is already ingested, **When** the same book is uploaded again, **Then** the system detects duplicates and skips re-embedding
2. **Given** a user asks multiple similar questions, **When** the questions are asked, **Then** retrieval results are cached when appropriate to reduce API calls
3. **Given** embedding generation, **When** the service responds, **Then** the system logs API usage to track quota consumption

---

### Edge Cases

- What happens when the book file is corrupted or password-protected?
- How does the system handle network timeouts during embedding generation?
- What happens when the vector database (Qdrant/Bonsai) is unavailable?
- How does the system handle concurrent ingestion requests from multiple administrators?
- What happens when a user's session expires during a long conversation?
- How does the system handle malformed or non-text content in PDF files?
- What happens when the free-tier API quota is exhausted mid-operation?
- How does the system handle very short or very long user questions?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST ingest book files (PDF, TXT, EPUB) by extracting text content and splitting it into semantically meaningful chunks
- **FR-002**: System MUST generate vector embeddings for each chunk using the Bonsai/Qwen API
- **FR-003**: System MUST store vectors in Qdrant/Bonsai with metadata including source book, chapter, and position
- **FR-004**: System MUST retrieve the top-k most similar chunks (default k=5) based on semantic similarity to user questions
- **FR-005**: System MUST generate answers using the retrieved chunks as context via Qwen API
- **FR-006**: System MUST support selected-text mode where users can highlight specific text to limit retrieval scope
- **FR-007**: System MUST display visible Login and Signup UI forms with username/email and password fields
- **FR-008**: System MUST authenticate users via email/password and create user sessions
- **FR-009**: System MUST validate user credentials on login and create secure sessions
- **FR-010**: System MUST prevent unauthenticated access to the chatbot interface
- **FR-011**: Frontend MUST send user questions to backend API and display returned answers
- **FR-012**: Backend MUST return answers within 10 seconds for 90% of requests
- **FR-013**: System MUST deduplicate ingestion requests to avoid re-processing the same book
- **FR-014**: System MUST log all API calls and quota usage for monitoring
- **FR-015**: System MUST handle and log all errors during ingestion, retrieval, and answer generation
- **FR-016**: System MUST display user-friendly error messages for API failures, network errors, and quota exhaustion
- **FR-017**: System MUST support concurrent chunk embedding with rate limiting to respect API quotas
- **FR-018**: System MUST provide citation references showing which book sections were used in answers

### Non-Functional Requirements

#### Performance

- **NFR-001**: Book ingestion must complete within 5 minutes for a 200-page book on average network conditions
- **NFR-002**: Question-answering requests must return answers within 10 seconds for 90% of requests
- **NFR-003**: Retrieval queries must complete within 2 seconds across the knowledge base
- **NFR-004**: System must support at least 10 concurrent users without performance degradation

#### Security

- **NFR-005**: User passwords must be hashed using industry-standard algorithms (bcrypt/Argon2) before storage
- **NFR-006**: API keys for Bonsai/Qwen must be stored securely and never exposed in client-side code
- **NFR-007**: User sessions must expire after 30 minutes of inactivity
- **NFR-008**: All authentication requests must be rate-limited to prevent brute-force attacks
- **NFR-009**: System must sanitize all user inputs to prevent injection attacks

#### Reliability

- **NFR-010**: System must retry failed API calls up to 3 times with exponential backoff
- **NFR-011**: System must maintain data consistency - partial ingestion failures must be rolled back
- **NFR-012**: System must log all errors with sufficient context for debugging
- **NFR-013**: System must handle vector database unavailability gracefully with clear error messages

#### Quota Efficiency

- **NFR-014**: System must cache embedding results for identical text chunks to avoid duplicate API calls
- **NFR-015**: System must batch embedding requests where API supports it to reduce per-call overhead
- **NFR-016**: System must deduplicate book uploads to prevent re-embedding the same content
- **NFR-017**: System must log and track daily/monthly API usage to prevent quota exhaustion

#### Error Handling

- **NFR-018**: All error conditions must return structured error responses with user-friendly messages
- **NFR-019**: System must distinguish between transient errors (retryable) and permanent errors (user action required)
- **NFR-020**: System must provide guidance to users on how to resolve errors when appropriate

#### Logging & Observability

- **NFR-021**: System must log all API calls with timestamp, endpoint, request size, and response status
- **NFR-022**: System must log ingestion progress (chunks processed, embeddings generated, vectors stored)
- **NFR-023**: System must log retrieval metrics (query latency, number of chunks retrieved, answer generation time)
- **NFR-024**: System must log authentication events (login, logout, failed attempts)
- **NFR-025**: Logs must be structured and searchable for debugging and monitoring

### Key Entities

- **Book**: A complete document (PDF, TXT, EPUB) uploaded by an administrator containing the source material for the chatbot. Key attributes: title, file type, total pages/chapters, upload timestamp, ingestion status.
- **Chunk**: A semantically meaningful segment of a book (typically 500-1000 words) that is independently embedded and stored. Key attributes: text content, source book reference, chapter/section, position in book, embedding vector.
- **User**: A person who can log in and interact with the chatbot. Key attributes: username, email, hashed password, session information.
- **Session**: An authenticated user session allowing access to the chatbot. Key attributes: user reference, creation time, expiration time, activity timestamp.
- **Question**: A user-submitted query about the book content. Key attributes: question text, user reference, timestamp, selected-text context (if used).
- **Answer**: The system-generated response to a user question. Key attributes: answer text, question reference, retrieved chunks used, generation timestamp, citation references.
- **APIUsageLog**: A record of external API calls for quota tracking. Key attributes: API service, endpoint, timestamp, request size, response status, quota cost.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive answers to 95% of questions about ingested book content within 10 seconds
- **SC-002**: Book ingestion results in at least one indexed chunk per chapter (minimum 20 chunks for a typical 20-chapter book)
- **SC-003**: Login and Signup forms are visible to 100% of users attempting to access the application
- **SC-004**: API quota usage is reduced by at least 30% through caching and deduplication strategies
- **SC-005**: System successfully ingests 95% of book files on first attempt without manual intervention
- **SC-006**: Users can complete the signup and login process in under 2 minutes
- **SC-007**: 90% of answers include citation references to specific book sections
- **SC-008**: System handles at least 10 concurrent users with average response times under 10 seconds

## Assumptions

- Free-tier Bonsai/Qwen API provides sufficient quota for the expected usage volume (less than 1000 API calls per day)
- Qdrant vector database is available and accessible (either cloud or self-hosted)
- Book files are in common formats (PDF, TXT, EPUB) and not heavily protected with DRM or encryption
- Users have basic familiarity with web-based forms (login/signup)
- Network connectivity is reliable enough to support API calls within timeout thresholds
- Frontend and backend are deployed in environments with network connectivity to each other

## Dependencies

- Bonsai/Qwen API for embedding generation and answer generation
- Qdrant vector database for storing and retrieving embeddings
- Frontend authentication UI components (must be created/visible)
- Existing backend authentication system (must be integrated with visible UI)
- Book file parsing libraries for PDF, TXT, and EPUB formats

## Out of Scope

- Multi-book querying (answers from multiple books simultaneously)
- Real-time streaming of long answers
- Voice input/output
- Advanced RAG features like re-ranking, hybrid search, or retrieval refinement
- Admin dashboard for monitoring ingestion status (beyond basic logging)
- User profile management beyond authentication
- Social login (OAuth, SSO)
- Mobile app or native mobile support
