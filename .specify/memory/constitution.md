<!-- SYNC IMPACT REPORT
Version change: 1.0.1 -> 1.1.0
Modified principles: None (existing principles preserved)
Added sections: RAG Chatbot Specific Standards (comprehensive)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ aligned
- .specify/templates/spec-template.md ✅ aligned
- .specify/templates/tasks-template.md ✅ aligned
Runtime docs: README.md ✅ already documented
Follow-up TODOs: None
Merge source: constitution-rag.md and constitution-rag-chatbot.md
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Content Accuracy (NON-NEGOTIABLE)
All AI-related content MUST be factually accurate, properly sourced, and reviewed before publication. Information MUST be current, citing recent research and industry developments. All claims about AI capabilities MUST be supported by evidence and distinguish between theoretical possibilities and demonstrated realities. In the RAG context, all chatbot responses MUST be grounded in accurate, verified information from the textbook content.

**Rationale**: Educational content requires the highest standards of accuracy to build trust and provide reliable learning outcomes. Misinformation in AI education can lead to incorrect implementations and understanding.

### II. Educational Clarity
Content MUST be structured pedagogically with clear learning objectives, progressive complexity, and practical examples. Abstract concepts MUST be explained using accessible analogies and visual aids. Each chapter MUST include exercises, summaries, and next-step connections to maintain educational coherence. The chatbot interface MUST provide clear, understandable responses that align with this educational tone.

**Rationale**: Learning Physical AI and Robotics requires clear progression from fundamentals to advanced topics. Pedagogical structure ensures students can build knowledge systematically.

### III. Ethical Responsibility
AI ethics MUST be woven throughout all content, addressing bias, privacy, transparency, and societal impacts. Discussions of AI applications MUST include ethical considerations, potential negative consequences, and mitigation strategies. Responsible AI development practices MUST be emphasized over purely technical capabilities. The chatbot MUST embody responsible AI principles, avoiding harmful, biased, or inappropriate responses.

**Rationale**: Physical AI systems interact with the real world and humans directly. Ethical considerations are not optional but fundamental to safe and beneficial AI development.

### IV. Practical Application
Theoretical knowledge MUST be paired with hands-on examples, code samples, and realistic use cases. All concepts MUST be demonstrated through concrete implementations that readers can reproduce. Chapters MUST bridge the gap between academic understanding and industry application. The RAG system MUST maintain coherent conversation context while accurately retrieving relevant information from the textbook corpus.

**Rationale**: Robotics and Physical AI are applied disciplines. Students need practical skills, not just theoretical knowledge, to succeed in the field.

### V. Test-First Development (NON-NEGOTIABLE)
All new features MUST follow Test-Driven Development (TDD): Tests written → User approved → Tests fail → Then implement. Red-Green-Refactor cycle MUST be strictly enforced. All code changes MUST include corresponding tests. Integration tests MUST cover API contracts, authentication flows, RAG pipeline, and component interactions.

**Rationale**: Test-first development ensures reliability, reduces bugs, and provides living documentation. For educational software handling user data and AI interactions, testing is critical for trust and stability.

### VI. Accessibility and Inclusion
Content MUST be accessible to diverse audiences regardless of background, with clear language and supportive learning materials. Different learning styles MUST be accommodated through varied presentation formats (text, diagrams, code, exercises). Inclusive examples and case studies MUST be prioritized. The chatbot MUST serve as an effective educational tool that supplements, not replaces, the textbook content, guiding users with diverse needs effectively.

**Rationale**: Quality education should be available to all learners. Accessibility is both a moral imperative and expands the reach and impact of the textbook.

### VII. System Reliability & Observability
All technical components MUST maintain consistent availability and performance that enable productive learning experiences. Error handling MUST be graceful, with helpful messages when technical issues occur. The system MUST be resilient to various user inputs while maintaining safety and accuracy standards. Structured logging MUST be implemented for debugging and monitoring. Both the content management and RAG systems MUST be designed with scalability and maintainability in mind.

**Rationale**: Educational platforms must be reliable. Students expect consistent access to materials. Observability enables rapid issue detection and resolution.

## Technology Stack Standards

### Backend
- **Framework**: FastAPI with Python 3.9+ for high-performance async APIs
- **Vector Database**: Qdrant for semantic search and RAG retrieval
- **Embeddings**: Cohere API for document and query embeddings
- **LLM**: OpenRouter (Gemini 1.5 Flash) for chatbot responses
- **Database**: SQLite for development, PostgreSQL for production
- **ORM**: SQLAlchemy for database interactions
- **Authentication**: JWT tokens with bcrypt password hashing
- **Testing**: pytest with async support

### Frontend
- **Framework**: React 18 with Docusaurus 3 for documentation
- **Authentication**: better-auth library for client-side auth
- **Server**: Express.js for production serving with backend proxy
- **Testing**: Jest and React Testing Library
- **Build**: Babel for transpilation, webpack for bundling

### Deployment
- **Frontend**: Netlify with GitHub auto-deployment
- **Backend**: Dockerized deployment on Hugging Face Spaces
- **Environment**: Separate .env files for frontend and backend configurations

## Quality Standards

All content MUST meet high editorial standards with consistent terminology, proper citations, and professional formatting. Technical content MUST be verified for correctness and reproducibility. All code examples MUST be tested against current library versions and include error handling guidance. The RAG system MUST achieve high precision in information retrieval and relevance of responses, with continuous monitoring and improvement mechanisms.

### Code Quality Requirements
- All code MUST follow language-specific style guides (PEP 8 for Python, Airbnb for JavaScript)
- Functions MUST have clear purposes and be under 50 lines when possible
- Complex logic MUST include inline comments explaining the "why"
- All public APIs MUST have comprehensive docstrings/JSDoc
- No hardcoded secrets or credentials in source code
- All environment-specific configuration MUST use environment variables

## Development Workflow

All contributions MUST undergo technical review by domain experts and editorial review for consistency with style guidelines. Major content changes MUST have outline approval before full implementation. Peer review and reader feedback mechanisms MUST be established for continuous improvement. All RAG system implementations MUST undergo rigorous testing with sample queries covering textbook content breadth and depth.

### Spec-Driven Development Process
1. **Specification**: All features MUST start with a spec.md defining user stories, requirements, and success criteria
2. **Planning**: Architecture plan (plan.md) MUST be created identifying technical approach, APIs, and data models
3. **Tasks**: Actionable tasks (tasks.md) MUST be generated with clear acceptance criteria and test cases
4. **Implementation**: Follow Red-Green-Refactor TDD cycle for all code changes
5. **Review**: Code review MUST verify spec compliance, test coverage, and constitution adherence
6. **Documentation**: All features MUST update relevant documentation and ADRs for significant decisions

### Git Workflow
- Feature branches named descriptively (e.g., `add-auth-validation`, `fix-rag-timeout`)
- Conventional commit messages: `type(scope): subject` (feat, fix, chore, refactor, docs, test)
- Pull requests MUST include description, related issues, testing performed
- All PRs MUST pass CI checks and require one approval before merge
- Main branch MUST always be deployable

## Security Requirements

- All user passwords MUST be hashed with bcrypt (minimum 12 rounds)
- JWT tokens MUST expire and use secure secrets (minimum 32 characters)
- API endpoints MUST validate and sanitize all user inputs
- SQL queries MUST use parameterized statements (no string concatenation)
- Sensitive data MUST NOT be logged or exposed in error messages
- CORS MUST be configured restrictively for production
- HTTPS MUST be enforced in production environments
- Regular dependency updates MUST be performed to patch security vulnerabilities

## Performance Standards

- Frontend initial load MUST be under 3 seconds on 3G connection
- Backend API responses MUST be under 500ms for p95 latency (excluding LLM calls)
- RAG retrieval MUST return context within 2 seconds
- Chatbot responses MUST stream for improved perceived performance
- Database queries MUST use indexes for frequently accessed data
- Vector search MUST be optimized for sub-second retrieval on collections under 100k documents

## RAG Chatbot Specific Standards

### Purpose and Scope

The RAG (Retrieval-Augmented Generation) Chatbot provides an intelligent conversational interface that enables users to interact with the Physical AI & Humanoid Robotics Textbook content through natural language queries. The system retrieves relevant information from the textbook corpus and generates contextually appropriate responses that maintain educational accuracy and enhance the learning experience.

### RAG System Rules

1. **Source Fidelity**: The system MUST only provide information directly sourced from the textbook content or related course materials
2. **Citation Clarity**: Responses MUST clearly indicate when information is being retrieved from specific chapters or sections
3. **Uncertainty Acknowledgment**: The chatbot MUST acknowledge when queries cannot be answered with available content
4. **Educational Tone**: All generated responses MUST maintain the educational tone and accuracy of the textbook
5. **Clarification Over Guessing**: The system MUST handle ambiguous queries by asking for clarification rather than guessing
6. **Privacy Protection**: User data and conversations MUST be handled according to privacy requirements
7. **Graceful Degradation**: The system MUST gracefully handle technical errors and unavailable content
8. **Hallucination Prevention**: Generated content MUST be free of hallucinations not supported by the textbook

### RAG Architecture Components

- **Chat Interface**: React-based chat interface integrated with Docusaurus
- **Query Processing**: FastAPI backend handling query processing and response generation
- **Retrieval Engine**: Qdrant vector database for semantic search across textbook content
- **LLM Interface**: OpenRouter API integration with Gemini 1.5 Flash for response generation
- **Content Pipeline**: Text processing pipeline to convert textbook content to searchable format
- **Embedding Model**: Cohere API for generating query and document embeddings

### RAG Data Pipeline

1. **Content Extraction**: Textbook content extraction from Markdown files in `frontend/docs/`
2. **Chunking**: Content chunking into semantically meaningful segments (target: 500-1000 tokens)
3. **Embedding Generation**: Vector embeddings using Cohere models for semantic search
4. **Indexing**: Indexing into Qdrant with metadata (chapter, section, topic tags)
5. **Updates**: Regular updates to reflect content changes in textbook
6. **Quality Assurance**: QA checks to ensure accuracy of indexed content and retrieval relevance

### RAG Quality Metrics

- **Retrieval Precision**: >85% of retrieved chunks MUST be relevant to user query
- **Response Accuracy**: >90% of generated responses MUST be factually correct when verified against textbook
- **Response Time**: <3 seconds for query processing and response generation (p95)
- **Context Coherence**: Multi-turn conversations MUST maintain context across >5 exchanges
- **Citation Accuracy**: 100% of citations MUST correctly reference textbook sections

### RAG Security & Privacy

- **No Personal Data Retention**: User queries and conversations MUST NOT be logged or stored beyond session
- **API Rate Limiting**: Rate limiting to prevent abuse (100 requests/hour per user)
- **Input Sanitization**: All user inputs MUST be sanitized to prevent injection attacks
- **Secure Credentials**: API keys for Cohere, OpenRouter, Qdrant stored in environment variables
- **Encryption**: All data in transit MUST use HTTPS/TLS encryption

### RAG Deployment Standards

- **Containerization**: Backend API containerized with Docker for consistent deployment
- **Scalability**: System MUST handle 100 concurrent users without degradation
- **Monitoring**: Real-time monitoring of query latency, error rates, and response quality
- **Logging**: Structured logging for debugging (query patterns, retrieval results, error traces)
- **Backup**: Vector database backups performed daily to prevent data loss

### RAG Non-Goals

1. The system will NOT generate content that is not based on the textbook materials
2. The chatbot will NOT provide real-time updates outside of the textbook content
3. General AI knowledge not related to the textbook content is NOT within scope
4. The system will NOT replace the need to read the full textbook content
5. Advanced conversational AI features like personality or humor are NOT priorities
6. The system will NOT provide code debugging assistance beyond textbook examples

## Governance

This constitution governs all content creation, review, maintenance, code development, and RAG system activities for the Physical AI & Humanoid Robotics Textbook project. All contributors MUST affirm understanding of these principles before participating. Changes to this constitution require consensus among core maintainers and MUST consider impact on all existing content, system capabilities, and development workflows. Regular compliance reviews ensure adherence to stated principles.

### Amendment Process
1. Propose amendment with rationale and impact analysis
2. Review by core maintainers
3. Update constitution version according to semantic versioning:
   - **MAJOR**: Backward-incompatible governance or principle changes
   - **MINOR**: New principles or materially expanded guidance
   - **PATCH**: Clarifications, wording, or non-semantic refinements
4. Update dependent templates and documentation
5. Communicate changes to all contributors

### Compliance Review
- Constitution compliance MUST be verified in all pull request reviews
- Quarterly audits MUST check adherence to principles across codebase
- Violations MUST be documented and remediated within one sprint
- Complexity additions MUST be justified against simplicity principles

**Version**: 1.1.0 | **Ratified**: 2025-12-27 | **Last Amended**: 2025-12-27
