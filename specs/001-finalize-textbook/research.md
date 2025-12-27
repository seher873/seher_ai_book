# Research Findings: Finalize textbook Phase-1

## Overview

This document captures research findings relevant to implementing assessments, lab architecture, hardware alignment, glossary, and appendices for the AI textbook. This research addresses all previously identified "NEEDS CLARIFICATION" items from the technical context.

## Research Areas & Findings

### 1. Assessment Question Formats for Textbook Content

**Decision**: Use multiple-choice questions (MCQs) with detailed explanations for each answer option.

**Rationale**: MCQs are ideal for textbook assessments as they allow for immediate feedback and are easy to grade automatically. The detailed explanations for both correct and incorrect options promote learning by helping students understand why certain answers are right or wrong.

**Alternatives considered**: 
- Short answer questions: More difficult to grade automatically and may have subjective interpretations
- Essay questions: Require manual grading and are not suitable for immediate feedback
- Interactive coding challenges: More complex to implement in a static textbook format

### 2. Lab Architecture for AI Textbook

**Decision**: Implement a cloud-based Jupyter notebook environment (such as Google Colab or AWS SageMaker Notebooks).

**Rationale**: Cloud-based notebooks offer several advantages:
- No local installation required
- Consistent environment for all users
- Pre-configured with required libraries
- Easy access from anywhere
- Scalable computing resources for different complexity levels

**Alternatives considered**:
- Local Jupyter installations: Prone to environment setup issues and inconsistencies
- Docker containers: Reduces issues but still requires local setup and understanding of Docker
- Standalone applications: Would require separate development and maintenance efforts

### 3. Hardware Requirements Documentation

**Decision**: Document both minimum and recommended hardware requirements with specific benchmarks for different AI exercises.

**Rationale**: Clear hardware requirements help students prepare appropriately. Minimum requirements ensure the exercises run, while recommended requirements optimize the learning experience. Specific benchmarks (e.g., training time for simple models) provide concrete metrics.

**Alternatives considered**:
- Generic requirements: Often too vague to be helpful
- Exact hardware recommendations: Would become outdated quickly
- No hardware guidance: Would lead to student frustration with environment issues

### 4. Glossary Maintenance Strategy

**Decision**: Implement a centralized authority with scheduled quarterly updates combined with community feedback channels.

**Rationale**: Quarterly updates ensure the glossary stays current with the rapidly evolving field of AI. Centralized authority maintains consistency and quality. Community feedback helps identify needed additions and corrections between formal updates.

**Alternatives considered**:
- Ad-hoc updates: Could lead to inconsistencies
- Monthly updates: Might be too frequent given review processes
- No formal maintenance: Glossary would become outdated quickly

### 5. Appendix Organization

**Decision**: Structure appendices by topic/skill level with cross-references to main content.

**Rationale**: Topic-based organization allows students to find relevant supplementary material easily. Skill-level organization helps students gauge the complexity of additional content. Cross-references connect supplementary material to the main content for better learning flow.

**Alternatives considered**:
- Chronological organization: Less intuitive for reference purposes
- Alphabetical organization: Doesn't account for skill progression
- Random access format: Lacks pedagogical structure

### 6. Content Integration with Docusaurus Documentation System

**Decision**: Use MDX and Docusaurus plugins for interactive assessments and proper linking between textbook content, glossary, and appendices.

**Rationale**: This approach maintains compatibility with the existing documentation system while adding necessary interactivity for assessments. MDX allows for React components within Markdown, enabling dynamic features.

**Alternatives considered**:
- Separate system for assessments: Would fragment the learning experience
- Static links only: Would limit interactivity and immediate feedback
- Custom web application: Significant development overhead not justified

### 7. Accessibility and Inclusion Standards

**Decision**: Follow WCAG 2.1 AA guidelines for all content with special attention to code examples and mathematical equations.

**Rationale**: Following established accessibility guidelines ensures the textbook is usable by students with diverse needs. Special attention to code examples and equations addresses common accessibility challenges in technical content.

**Alternatives considered**:
- Basic accessibility compliance: May not address all learning needs
- More stringent AAA guidelines: Would increase development time significantly
- Self-defined accessibility standards: Lack proven effectiveness

## Technology Stack

Based on research findings, the following technology stack was confirmed:

- **Documentation Platform**: Docusaurus v3.x with React components
- **Content Format**: Markdown/CommonMark with MDX for interactivity
- **Development Environment**: Node.js v18+ for compatibility
- **Version Control**: Git with conventional commit messages
- **Infrastructure**: Cloud hosting for notebooks with CDN for static content