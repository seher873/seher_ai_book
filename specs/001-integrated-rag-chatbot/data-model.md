# Data Model: Integrated Read-Only RAG Chatbot for Physical AI Textbook

**Date**: 2025-12-16  
**Related Document**: [Implementation Plan](plan.md)  
**Entities**: Core data structures for the RAG chatbot system

## Overview

This document defines the data models for the RAG chatbot system based on the feature specification requirements. All models are designed to support the book-first principle, mandatory citations, privacy-safe operations, and zero content mutation rights.

## Core Entities

### 1. ChatSession

Represents a single conversation between user and chatbot, including query history and metadata stored in a serverless PostgreSQL database; data is deleted after session ends (privacy-safe).

**Fields**:
- `id` (string, UUID): Unique identifier for the session
- `session_token` (string): Temporary session identifier for anonymous users
- `created_at` (timestamp): Session start time
- `last_activity_at` (timestamp): Time of last interaction
- `expires_at` (timestamp): Expiration time for automatic cleanup
- `metadata` (JSON): Additional session data (optional)

**Validation Rules**:
- `id` must be a valid UUID
- `created_at` <= `last_activity_at` <= `expires_at`
- `expires_at` must be within 24 hours of `created_at`
- `session_token` must be unique and securely generated

**State Transitions**:
- `active` → `expired` (when `expires_at` is reached)
- `active` → `closed` (when user explicitly ends session)

### 2. Query

A user's input text to the chatbot, which triggers RAG retrieval and response generation using an AI agent framework.

**Fields**:
- `id` (string, UUID): Unique identifier for the query
- `session_id` (string): Foreign key linking to ChatSession
- `content` (string): The user's input text
- `query_type` (enum: "global", "focused"): Whether query targets full corpus or user-selected text
- `selected_text` (string, optional): Specific text selected by user for focused queries
- `timestamp` (timestamp): When the query was submitted
- `source_chunk_ids` (array of strings): IDs of knowledge chunks used for response

**Validation Rules**:
- `session_id` must reference an active ChatSession
- `content` must not be empty or exceed 1000 characters
- `query_type` must be either "global" or "focused"
- If `query_type` is "focused", `selected_text` must not be empty
- `source_chunk_ids` elements must be valid knowledge chunk IDs

**State Transitions**:
- `created` → `processing` → `completed` (with success or error)

### 3. Response

The chatbot's output containing information grounded in Physical AI textbook content with mandatory citations, generated via backend service.

**Fields**:
- `id` (string, UUID): Unique identifier for the response
- `query_id` (string): Foreign key linking to Query
- `content` (string): The chatbot's response text
- `citations` (array of Citation objects): Mandatory source references
- `timestamp` (timestamp): When the response was generated
- `token_usage` (object): API token counts for monitoring

**Validation Rules**:
- `query_id` must reference an existing Query
- `content` must not be empty
- `citations` array must not be empty (mandatory citation requirement)
- Each citation must reference a valid textbook section
- `token_usage` must contain `input_tokens` and `output_tokens`

**State Transitions**:
- `generated` → `delivered` (when sent to client)
- `generated` → `error` (if delivery fails)

### 4. Citation

Represents a reference to specific textbook content as required by the mandatory citation requirement.

**Fields**:
- `id` (string, UUID): Unique identifier
- `chapter_number` (string): The textbook chapter reference
- `section_number` (string): The specific section within the chapter
- `text_excerpt` (string): Excerpt of the cited text
- `page_reference` (string, optional): Page number if applicable
- `url` (string, optional): Direct link to the cited section in the textbook

**Validation Rules**:
- `chapter_number` and `section_number` must follow textbook's numbering scheme
- `text_excerpt` must not be empty
- `url` must be a valid internal reference to textbook content
- All citations must reference actual textbook content (no fabricated references)

### 5. KnowledgeChunk

Segments of Physical AI textbook content indexed in a vector database for retrieval, linked to specific chapters/sections.

**Fields**:
- `id` (string): Unique identifier for the knowledge chunk
- `chapter_number` (string): The textbook chapter this chunk belongs to
- `section_number` (string): The specific section within the chapter
- `content` (string): The actual text content of this chunk
- `embedding_vector` (array of floats): The vector representation for semantic search
- `metadata` (JSON): Additional info like page numbers, subsections, etc.
- `created_at` (timestamp): When this chunk was indexed

**Validation Rules**:
- `id` must be unique
- `chapter_number` and `section_number` must follow textbook's scheme
- `content` must not be empty and should be within 500-2000 tokens
- `embedding_vector` must have the correct dimensionality (e.g., 1536 for OpenAI text-embedding-3-small)
- `content` must be an exact excerpt from the textbook (no modifications)

**State Transitions**:
- `indexed` → `deprecated` (when textbook content is updated)

### 6. BookCorpus

The complete collection of Physical AI textbook content indexed as an immutable source for RAG operations (no overwrite, no mutation, no auto-editing).

**Fields**:
- `id` (string): Unique identifier for the corpus
- `name` (string): Name of the textbook
- `version` (string): Version identifier
- `total_chunks` (integer): Number of knowledge chunks in corpus
- `indexing_date` (timestamp): When the corpus was last indexed
- `checksum` (string): Integrity verification for the corpus

**Validation Rules**:
- `id` must be unique
- `name` must match the expected textbook name
- `version` must follow semantic versioning
- `total_chunks` must be >= 0
- `checksum` must match the actual corpus content

## Relationships

```
ChatSession (1) ←→ (Many) Query
Query (1) ←→ (1) Response
Response (1) ←→ (Many) Citation
KnowledgeChunk (Many) ←→ (Many) Response (via source_chunk_ids)
BookCorpus (1) ←→ (Many) KnowledgeChunk
```

## Constraints

### Primary Constraints
1. **Immutability**: KnowledgeChunks and BookCorpus entities must never be modified, only created or deprecated.
2. **Mandatory Citations**: Every Response must include at least one valid Citation.
3. **Privacy**: ChatSession data must be deleted after expiration.
4. **Content Authority**: All responses must be grounded in the BookCorpus content only.

### Data Integrity
1. Foreign key constraints between related entities
2. Content validation to prevent hallucinations
3. Proper indexing for efficient retrieval operations
4. Check constraints to enforce business rules

## State Management

### Session Management
- Automatic expiration and cleanup of inactive sessions
- Secure session token generation and validation
- Privacy-safe data handling without personal information

### Content Management
- Versioned textbook content with proper change tracking
- Immutable knowledge chunks with deprecation instead of modification
- Integrity verification mechanisms for corpus content

## Validation and Compliance

All data models must comply with the following requirements from the feature specification:
- FR-003: Accurate retrieval of textbook content
- FR-004: Mandatory source citations
- FR-008: Session management
- FR-010: Prevention of hallucinations
- FR-011: Source citations with chapter/section references
- FR-017: 95% accuracy
- FR-019: Mandatory source citations with chapter/section references
- FR-020: No modification of textbook content
- FR-021: No retention of user data beyond session
- FR-022: Privacy-safe operations

## Indexing Considerations

### Database Indexes
- Sessions table: Index on `expires_at` for efficient cleanup
- Queries table: Index on `session_id` and `timestamp`
- Responses table: Index on `query_id` and `timestamp`
- Citations table: Index on `chapter_number` and `section_number`

### Vector Database Indexes
- Knowledge chunks: Indexed by embedding vectors for semantic search
- Metadata indexes for efficient filtering by chapter/section