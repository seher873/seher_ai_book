# Data Model: Glossary and Appendix B (Reserved: Chatbot/RAG)

**Feature**: 006-glossary-appendix
**Date**: 2025-12-16

## Overview

This document defines the data structures for the glossary and appendices components of the AI textbook. The models are designed to work with Markdown content within the Docusaurus documentation system.

## GlossaryTerm Entity

### Definition
A GlossaryTerm represents a single AI concept or terminology with its definition and related information.

### Attributes
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier for the term |
| term | string | Yes | The actual term being defined |
| definition | string | Yes | The definition of the term |
| category | string | No | Topic category (e.g., "algorithms", "ethics", "tools") |
| relatedTerms | array of strings | No | Array of related term IDs |
| examples | array of strings | No | Array of examples that illustrate the term |
| synonyms | array of strings | No | Array of alternative terms with similar meanings |
| seeAlso | array of strings | No | Array of references to related concepts in the textbook |
| createdAt | datetime | No | Timestamp of creation |
| updatedAt | datetime | No | Timestamp of last update |

### Relationships
- Zero or more cross-references to other GlossaryTerm entities
- Zero or more references to AppendixSection entities
- Zero or more references to textbook modules

### Validation Rules
- term field is required and must be unique across all terms
- definition field is required with minimum 10 characters
- if provided, category must be from a predefined list of valid categories
- examples should provide substantive educational content

### Example
```markdown
---
id: "artificial-intelligence"
term: "Artificial Intelligence (AI)"
category: "fundamentals"
relatedTerms: ["machine-learning", "neural-networks"]
seeAlso: ["module-1", "module-2"]
---

# Artificial Intelligence (AI)

**Definition**: The simulation of human intelligence processes by machines, especially computer systems. These processes include learning, reasoning, and self-correction.

**Examples**:
- Image recognition systems
- Natural language processing
- Expert systems

**Synonyms**: 
- Machine intelligence
- Computational intelligence
```

## AppendixSection Entity

### Definition
An AppendixSection represents a detailed supplementary content section organized by topic.

### Attributes
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier for the appendix section |
| title | string | Yes | Title of the appendix section |
| content | string | Yes | Main content of the appendix (Markdown format) |
| category | string | No | Topic category (e.g., "tools", "mathematical-foundations", "datasets", "chatbot-rag") |
| skillLevel | string | No | Target skill level (beginner, intermediate, advanced) |
| relatedModules | array of strings | No | Array of module IDs this appendix connects to |
| tags | array of strings | No | Array of tags for easy search |
| references | array of strings | No | Array of references cited in the appendix |
| createdAt | datetime | No | Timestamp of creation |
| updatedAt | datetime | No | Timestamp of last update |

### Relationships
- Zero or more cross-references to GlossaryTerm entities
- Zero or more references to textbook modules
- Zero or more references to other AppendixSection entities

### Validation Rules
- title is required and must be unique within the same category
- content must follow Markdown format specifications
- if provided, skillLevel must be one of "beginner", "intermediate", or "advanced"
- references should follow standard academic citation format if possible

### Example
```markdown
---
id: "appendix-b-chatbot-rag"
title: "Appendix B: Chatbot and RAG Technologies"
category: "chatbot-rag"
skillLevel: "advanced"
relatedModules: ["module-4", "module-5"]
tags: ["chatbot", "rag", "retrieval-augmented-generation", "nlp"]
references: ["Lewis et al. 2020", "Brown et al. 2020"]
---

# Appendix B: Chatbot and RAG Technologies

## Overview

This appendix provides detailed information about modern chatbot technologies and Retrieval-Augmented Generation (RAG) systems...

[Detailed content continues in Markdown format]
```

## CrossReference Entity

### Definition
A CrossReference represents a mapping between terms in the glossary and related content in the appendices or main textbook.

### Attributes
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier for the cross-reference |
| sourceType | string | Yes | Type of source entity (glossaryTerm, appendixSection, module) |
| sourceId | string | Yes | ID of the source entity |
| targetType | string | Yes | Type of target entity (glossaryTerm, appendixSection, module) |
| targetId | string | Yes | ID of the target entity |
| relationshipType | string | Yes | Type of relationship (related, seeAlso, prerequisite, etc.) |
| createdAt | datetime | No | Timestamp of creation |

### Relationships
- Links exactly two entities (source and target) of specified types

### Validation Rules
- Both sourceId and targetId must reference existing entities
- sourceType and targetType must be from a predefined list of valid types
- relationshipType must be from a predefined list of valid relationships

### Example
```json
{
  "id": "xr-001",
  "sourceType": "glossaryTerm",
  "sourceId": "artificial-intelligence",
  "targetType": "appendixSection",
  "targetId": "appendix-b-chatbot-rag",
  "relationshipType": "related",
  "createdAt": "2025-12-16T10:00:00Z"
}
```

## NavigationLink Entity

### Definition
A NavigationLink enables users to move between main content, glossary, and appendices.

### Attributes
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier for the navigation link |
| label | string | Yes | Text to display for the link |
| url | string | Yes | URL path to the target location |
| sourceSection | string | Yes | Section where the link is placed (mainContent, glossary, appendices) |
| targetSection | string | Yes | Section the link navigates to (mainContent, glossary, appendices) |
| displayPriority | number | No | Priority for display order (lower numbers first) |
| createdAt | datetime | No | Timestamp of creation |

### Validation Rules
- url must be a valid relative path within the documentation site
- sourceSection and targetSection must be from a predefined list of valid sections
- displayPriority must be a positive integer if provided

### Example
```json
{
  "id": "nav-001",
  "label": "View Glossary",
  "url": "/docs/glossary",
  "sourceSection": "mainContent",
  "targetSection": "glossary",
  "displayPriority": 1,
  "createdAt": "2025-12-16T10:00:00Z"
}
```