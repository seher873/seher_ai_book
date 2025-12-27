# Data Model: Finalize textbook Phase-1

## Overview

This document defines the data models for the new components of the AI textbook: assessments, lab architecture, hardware alignment, glossary, and appendices.

## Core Entities

### 1. Assessment Questions

**Entity Name**: AssessmentQuestion
- **Fields**:
  - id: unique identifier for the question
  - moduleId: reference to the module/chapter this question relates to
  - questionText: the actual question text
  - questionType: type of question (currently "multiple-choice")
  - options: array of possible answer options
  - correctAnswer: index of the correct answer option
  - explanation: detailed explanation of the correct answer
  - incorrectExplanations: array of explanations for incorrect options
  - difficultyLevel: level of difficulty (beginner, intermediate, advanced)
  - tags: array of tags for categorizing the question (e.g., "neural-networks", "ethics")
  - createdAt: timestamp of creation
  - updatedAt: timestamp of last update

**Relationships**: 
- Belongs to one Module/Chapter
- Related to zero or more similar questions via tags

**Validation Rules**:
- questionText is required and must not exceed 5000 characters
- options array must have at least 2 items and no more than 10
- correctAnswer must be a valid index in options array
- explanation is required and must provide substantive content
- tags must consist of valid, pre-approved terms

### 2. Lab Exercise

**Entity Name**: LabExercise
- **Fields**:
  - id: unique identifier for the lab exercise
  - title: title of the exercise
  - description: brief description of what the exercise covers
  - prerequisites: array of module IDs or skills required
  - estimatedDuration: time estimate in minutes
  - notebookUrl: URL to the cloud-based notebook environment
  - objectives: array of learning objectives
  - steps: ordered array of instructions for the exercise
  - datasets: array of dataset names or URLs used in the exercise
  - hardwareRequirements: reference to hardware profile
  - difficultyLevel: level of difficulty
  - tags: array of tags for categorizing the exercise
  - createdAt: timestamp of creation
  - updatedAt: timestamp of last update

**Relationships**:
- Connects to one or more Modules/Chapters
- Associated with one HardwareProfile
- Contains one or more CodeSnippets

**Validation Rules**:
- title and description are required
- estimatedDuration must be positive
- notebookUrl must be a valid cloud notebook URL
- objectives array must not be empty
- steps array must not be empty
- hardwareRequirements must refer to an existing hardware profile

### 3. Hardware Profile

**Entity Name**: HardwareProfile
- **Fields**:
  - id: unique identifier for the hardware profile
  - name: descriptive name of the profile
  - cpuRequirements: description of CPU requirements
  - gpuRequirements: description of GPU requirements (optional)
  - memoryRequirements: amount of RAM required
  - storageRequirements: amount of disk space required
  - networkRequirements: network requirements if any
  - performanceBenchmarks: reference hardware with benchmark times
  - tags: array of tags for categorizing the exercises that use this profile
  - createdAt: timestamp of creation
  - updatedAt: timestamp of last update

**Relationships**:
- Used by zero or more LabExercises
- Referenced by zero or more Modules/Chapters

**Validation Rules**:
- name is required and unique
- memoryRequirements and storageRequirements must be positive values
- performanceBenchmarks must include comparable hardware and time measures

### 4. Glossary Term

**Entity Name**: GlossaryTerm
- **Fields**:
  - id: unique identifier for the term
  - term: the actual term being defined
  - definition: the definition of the term
  - category: topic category (e.g., "algorithms", "ethics", "tools")
  - relatedTerms: array of related term IDs
  - examples: array of examples that illustrate the term
  - synonyms: array of alternative terms with similar meanings
  - seeAlso: array of references to related concepts in the textbook
  - lastReviewed: date of last review and validation
  - createdAt: timestamp of creation
  - updatedAt: timestamp of last update

**Relationships**:
- Connected to zero or more other GlossaryTerms via relatedTerms
- Referenced by zero or more Modules/Chapters

**Validation Rules**:
- term and definition are required
- term must be unique (case-insensitive)
- category must be from a predefined list
- relatedTerms must refer to existing glossary terms

### 5. Appendix

**Entity Name**: Appendix
- **Fields**:
  - id: unique identifier for the appendix
  - title: title of the appendix
  - content: main content of the appendix (Markdown format)
  - category: topic category (e.g., "tools", "mathematical-foundations", "datasets")
  - skillLevel: target skill level (beginner, intermediate, advanced)
  - relatedModules: array of module IDs this appendix connects to
  - tags: array of tags for easy search
  - references: array of references cited in the appendix
  - lastUpdated: date of last significant update
  - createdAt: timestamp of creation
  - updatedAt: timestamp of last update

**Relationships**:
- Connected to zero or more Modules/Chapters via relatedModules
- May reference zero or more GlossaryTerms

**Validation Rules**:
- title and content are required
- skillLevel must be one of the predefined levels
- category must be from a predefined list
- relatedModules must refer to existing modules