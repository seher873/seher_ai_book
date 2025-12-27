# Data Model: Physical AI Textbook

## Overview
This document defines the data structures and content models for the Physical AI & Humanoid Robotics textbook. Rather than traditional software data models, this focuses on content organization and relationships.

## Content Entities

### Chapter
**Description**: Main content unit of the textbook
- **Fields**:
  - id: Unique identifier (e.g., "ch1", "ch2")
  - title: Chapter title
  - purpose: Educational purpose statement
  - learning_outcomes: List of specific learning outcomes
  - subsections: List of subsections within the chapter
  - prerequisites: Prerequisites required for understanding
  - estimated_time: Estimated time to complete (hours)
  - diagrams: List of diagram placeholders with descriptions
  - exercises: List of exercise types and numbers
  - summary: Chapter summary
  - next_steps: Connections to subsequent chapters

**Relationships**:
- Contains multiple Subsections
- Contains multiple Exercises
- Contains multiple Diagrams
- References other Chapters (prerequisites, next_steps)

### Subsection
**Description**: Component of a Chapter providing focused content
- **Fields**:
  - id: Unique identifier (e.g., "ch1-1", "ch1-2")
  - title: Subsection title
  - content: Main content in markdown format
  - objectives: Specific learning objectives for this subsection
  - examples: List of examples provided
  - concepts: List of key concepts covered
  - difficulty: Difficulty level (beginner, intermediate, advanced)

**Relationships**:
- Belongs to one Chapter
- Contains multiple Examples
- Covers multiple Concepts

### Module
**Description**: Technical deep-dive content that supplements chapters
- **Fields**:
  - id: Unique identifier (e.g., "mod1", "mod2")
  - title: Module title
  - technical_topic: Specific technical topic covered
  - content: Detailed content in markdown format
  - prerequisites: Prerequisites required for understanding
  - tools: Required tools, software, or hardware
  - implementation_steps: Step-by-step implementation guide
  - best_practices: Industry best practices covered
  - common_pitfalls: Common mistakes and how to avoid them

**Relationships**:
- Supplements one or more Chapters
- Uses multiple Tools
- Contains multiple Implementation Steps

### Week (Curriculum Component)
**Description**: Weekly curriculum plan element
- **Fields**:
  - week_number: Week identifier (1-14)
  - goals: Learning goals for the week
  - covered_topics: List of topics covered
  - required_reading: Chapters and sections to read
  - skills: Skills students should acquire
  - labs: Lab assignments for the week
  - assessment: Assessment methods for the week
  - estimated_effort: Estimated student effort (hours)

**Relationships**:
- Connects to multiple Chapters
- Contains multiple Labs
- Covers multiple Skills

### Lab
**Description**: Hands-on exercises for practical application
- **Fields**:
  - id: Unique identifier (e.g., "lab1.1", "lab2.2")
  - title: Lab title
  - objective: Primary objective of the lab
  - required_equipment: Physical or virtual equipment needed
  - setup_instructions: How to set up for the lab
  - procedure: Step-by-step procedure
  - expected_outcomes: What students should achieve
  - troubleshooting: Common issues and solutions
  - evaluation_criteria: How the lab will be evaluated

**Relationships**:
- Associated with a Week
- Covers specific Chapter content
- Requires specific Equipment

### Hardware Recommendation
**Description**: Hardware components recommended for practical exercises
- **Fields**:
  - id: Unique identifier (e.g., "hw-001")
  - name: Product name
  - category: Category (robot_platform, computing, sensors, etc.)
  - level: Target level (entry, intermediate, advanced)
  - specifications: Technical specifications
  - cost_range: Estimated cost range
  - use_cases: Primary use cases in the textbook
  - pros: Advantages
  - cons: Disadvantages
  - alternatives: Alternative options
  - setup_guide: How to set up for textbook use

**Relationships**:
- Used in multiple Labs
- Supports specific Chapters

### Appendix
**Description**: Supplementary material
- **Fields**:
  - id: Unique identifier (e.g., "app-a", "app-b")
  - title: Appendix title
  - type: Type (glossary, troubleshooting, resources, etc.)
  - content: Main content in markdown format
  - related_chapters: Chapters that reference this appendix

**Relationships**:
- Referenced by multiple Chapters
- Contains Glossary Terms (for glossary appendix)

### Glossary Term
**Description**: Term defined in the glossary
- **Fields**:
  - id: Unique identifier (e.g., "term-actuator")
  - term: The term being defined
  - definition: Clear definition
  - related_terms: Related terms
  - chapter_first_mentioned: Chapter where term first appears

**Relationships**:
- Belongs to an Appendix
- May reference other Glossary Terms

### Code Example
**Description**: Code samples included in the textbook
- **Fields**:
  - id: Unique identifier
  - title: Description of the example
  - language: Programming language used
  - code: Code content (with formatting preserved)
  - purpose: What the code demonstrates
  - input: Sample input if applicable
  - output: Expected output if applicable
  - error_handling: How errors are handled
  - chapter: Chapter where example appears
  - license: License information if using external code

**Relationships**:
- Associated with a Chapter
- May reference other Code Examples