# Data Model: Module 2 Content Generation

## Entities

### Module 2 Content
- **id**: string (unique identifier)
- **title**: string (title of the content piece)
- **content**: string (Markdown content)
- **contentType**: enum (e.g., "lesson", "chapter", "exercise", "example")
- **authorId**: string (reference to author)
- **reviewerId**: string (reference to reviewer, optional)
- **status**: enum ("draft", "under_review", "approved", "published")
- **createdAt**: datetime
- **updatedAt**: datetime
- **version**: number (for version control)
- **learningObjectives**: array of strings (specific skills/concepts)
- **prerequisites**: array of strings (required knowledge)
- **handsOnExamples**: array of objects (interactive examples)
- **assessments**: array of assessment objects
- **chapterNumber**: number (for organization)

### Learning Objectives
- **id**: string (unique identifier)
- **description**: string (what students should acquire)
- **moduleId**: string (reference to the module)
- **associatedContentIds**: array of strings (content that teaches this objective)

### Assessment
- **id**: string (unique identifier)
- **type**: enum ("quiz", "project", "hands-on")
- **question**: string (the assessment question)
- **options**: array of strings (for multiple choice)
- **correctAnswer**: string/array (correct answer(s))
- **explanation**: string (explanation of the correct answer)
- **associatedContentId**: string (content this assessment tests)

### User
- **id**: string (unique identifier)
- **username**: string (unique)
- **email**: string (unique)
- **role**: enum ("author", "editor", "student", "admin")
- **createdAt**: datetime
- **lastLogin**: datetime

### Chapter
- **id**: string (unique identifier)
- **title**: string
- **moduleNumber**: number (should be 2 for this module)
- **chapterNumber**: number (sequence: 1-5)
- **chapterTitle**: string (e.g., "Introduction to Digital Twins", "Gazebo Fundamentals", etc.)
- **contentIds**: array of strings (IDs of content in this chapter)
- **learningObjectives**: array of strings
- **estimatedTime**: number (minutes to complete)

## Relationships
- A Module 2 Content entity belongs to one Chapter (via chapterNumber)
- A Chapter contains multiple Module 2 Content entities (via contentIds)
- A User can create/modify multiple Module 2 Content entities
- A Module 2 Content entity has one Author (User)
- A Module 2 Content entity may have one Reviewer (User)
- A Learning Objective can be associated with multiple Module 2 Content entities
- A Module 2 Content entity can have multiple Assessments

## Validation Rules
From requirements:

1. All content must be specifically relevant to Module 2 topics in an AI textbook (FR-001)
2. Content must align with educational learning objectives (FR-002)
3. Clear distinction between Module 2 content and other modules must be maintained (FR-005)
4. Content must be stored locally within the textbook system (FR-006)
5. All content must cover Digital Twin concepts with Gazebo and Unity (FR-009)
6. Content must target beginner to intermediate level academic audience (FR-010)
7. Content should follow 5-chapter structure (FR-011)
8. Content must include interactive text with hands-on examples (FR-012)

## State Transitions

### Content Status Transitions
- draft → under_review (when author submits for review)
- under_review → approved (when reviewer approves)
- under_review → draft (when reviewer requests changes)
- approved → published (when admin publishes)
- published → approved (when making changes to published content)
- approved → draft (when major changes needed)

### User Role Transitions
- student → author (when given author privileges)
- author → editor (when given editing privileges)
- editor → author (when privileges revoked)
- author → student (when privileges revoked)