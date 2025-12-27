---
id: 005
title: spec-analysis-report
stage: general
date: 2025-12-27
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: sp.analyze
labels: [analysis,spec,plan,tasks,constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /specs/001-finalize-textbook/spec.md
 - /specs/001-finalize-textbook/plan.md
 - /specs/001-finalize-textbook/tasks.md
 - /.specify/memory/constitution-full.md
tests:
 - analysis of cross-artifact consistency
---

## Prompt

---
description: Perform a non-destructive cross-artifact consistency and quality analysis across spec.md, plan.md, and tasks.md after task generation.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Goal

Identify inconsistencies, duplications, ambiguities, and underspecified items across the three core artifacts (`spec.md`, `plan.md`, `tasks.md`) before implementation. This command MUST run only after `/sp.tasks` has successfully produced a complete `tasks.md`.

## Operating Constraints

**STRICTLY READ-ONLY**: Do **not** modify any files. Output a structured analysis report. Offer an optional remediation plan (user must explicitly approve before any follow-up editing commands would be invoked manually).

**Constitution Authority**: The project constitution (`.specify/memory/constitution.md`) is **non-negotiable** within this analysis scope. Constitution conflicts are automatically CRITICAL and require adjustment of the spec, plan, or tasks—not dilution, reinterpretation, or silent ignoring of the principle. If a principle itself needs to change, that must occur in a separate, explicit constitution update outside `/sp.analyze`.

## Execution Steps

### 1. Initialize Analysis Context

Run `.specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks` once from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS. Derive absolute paths:

- SPEC = FEATURE_DIR/spec.md
- PLAN = FEATURE_DIR/plan.md
- TASKS = FEATURE_DIR/tasks.md

Abort with an error message if any required file is missing (instruct the user to run missing prerequisite command).
For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

### 2. Load Artifacts (Progressive Disclosure)

Load only the minimal necessary context from each artifact:

**From spec.md:**

- Overview/Context
- Functional Requirements
- Non-Functional Requirements
- User Stories
- Edge Cases (if present)

**From plan.md:**

- Architecture/stack choices
- Data Model references
- Phases
- Technical constraints

**From tasks.md:**

- Task IDs
- Descriptions
- Phase grouping
- Parallel markers [P]
- Referenced file paths

**From constitution:**

- Load `.specify/memory/constitution.md` for principle validation

### 3. Build Semantic Models

Create internal representations (do not include raw artifacts in output):

- **Requirements inventory**: Each functional + non-functional requirement with a stable key (derive slug based on imperative phrase; e.g., "User can upload file" → `user-can-upload-file`)
- **User story/action inventory**: Discrete user actions with acceptance criteria
- **Task coverage mapping**: Map each task to one or more requirements or stories (inference by keyword / explicit reference patterns like IDs or key phrases)
- **Constitution rule set**: Extract principle names and MUST/SHOULD normative statements

### 4. Detection Passes (Token-Efficient Analysis)

Focus on high-signal findings. Limit to 50 findings total; aggregate remainder in overflow summary.

#### A. Duplication Detection

- Identify near-duplicate requirements
- Mark lower-quality phrasing for consolidation

#### B. Ambiguity Detection

- Flag vague adjectives (fast, scalable, secure, intuitive, robust) lacking measurable criteria
- Flag unresolved placeholders (TODO, TKTK, ???, `<placeholder>`, etc.)

#### C. Underspecification

- Requirements with verbs but missing object or measurable outcome
- User stories missing acceptance criteria alignment
- Tasks referencing files or components not defined in spec/plan

#### D. Constitution Alignment

- Any requirement or plan element conflicting with a MUST principle
- Missing mandated sections or quality gates from constitution

#### E. Coverage Gaps

- Requirements with zero associated tasks
- Tasks with no mapped requirement/story
- Non-functional requirements not reflected in tasks (e.g., performance, security)

#### F. Inconsistency

- Terminology drift (same concept named differently across files)
- Data entities referenced in plan but absent in spec (or vice versa)
- Task ordering contradictions (e.g., integration tasks before foundational setup tasks without dependency note)
- Conflicting requirements (e.g., one requires Next.js while other specifies Vue)

### 5. Severity Assignment

Use this heuristic to prioritize findings:

- **CRITICAL**: Violates constitution MUST, missing core spec artifact, or requirement with zero coverage that blocks baseline functionality
- **HIGH**: Duplicate or conflicting requirement, ambiguous security/performance attribute, untestable acceptance criterion
- **MEDIUM**: Terminology drift, missing non-functional task coverage, underspecified edge case
- **LOW**: Style/wording improvements, minor redundancy not affecting execution order

### 6. Produce Compact Analysis Report

Output a Markdown report (no file writes) with the following structure:

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| A1 | Duplication | HIGH | spec.md:L120-134 | Two similar requirements ... | Merge phrasing; keep clearer version |

(Add one row per finding; generate stable IDs prefixed by category initial.)

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|

**Constitution Alignment Issues:** (if any)

**Unmapped Tasks:** (if any)

**Metrics:**

- Total Requirements
- Total Tasks
- Coverage % (requirements with >=1 task)
- Ambiguity Count
- Duplication Count
- Critical Issues Count

### 7. Provide Next Actions

At end of report, output a concise Next Actions block:

- If CRITICAL issues exist: Recommend resolving before `/sp.implement`
- If only LOW/MEDIUM: User may proceed, but provide improvement suggestions
- Provide explicit command suggestions: e.g., "Run /sp.specify with refinement", "Run /sp.plan to adjust architecture", "Manually edit tasks.md to add coverage for 'performance-metrics'"

### 8. Offer Remediation

Ask the user: "Would you like me to suggest concrete remediation edits for the top N issues?" (Do NOT apply them automatically.)

## Operating Principles

### Context Efficiency

- **Minimal high-signal tokens**: Focus on actionable findings, not exhaustive documentation
- **Progressive disclosure**: Load artifacts incrementally; don't dump all content into analysis
- **Token-efficient output**: Limit findings table to 50 rows; summarize overflow
- **Deterministic results**: Rerunning without changes should produce consistent IDs and counts

### Analysis Guidelines

- **NEVER modify files** (this is read-only analysis)
- **NEVER hallucinate missing sections** (if absent, report them accurately)
- **Prioritize constitution violations** (these are always CRITICAL)
- **Use examples over exhaustive rules** (cite specific instances, not generic patterns)
- **Report zero issues gracefully** (emit success report with coverage statistics)

## Context

my bacend dir is correct with python version and vertual envermint if not correct fix them properly Act as Ensure everything runs inside a Python virtual environment and all dependencies are installed there.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
## Response snapshot

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| D1 | Constitution Alignment | CRITICAL | constitution-full.md, spec.md | Missing core requirement for test-first development approach | Add TDD requirements to spec and tasks |
| D2 | Coverage Gap | HIGH | tasks.md | No tasks for performance requirements from spec | Add performance testing tasks to Phase 8 |
| D3 | Constitution Alignment | HIGH | tasks.md | No explicit accessibility implementation tasks for WCAG compliance | Add WCAG 2.1 AA compliance tasks |
| D4 | Inconsistency | MEDIUM | spec.md, plan.md | Hardware requirements terminology differs between documents | Standardize "minimum/recommended" vs "basic/advanced" requirements |
| D5 | Ambiguity | MEDIUM | spec.md | "95% of students successfully set up" lacks time constraint | Define success criteria timeframe |
| D6 | Underspecification | MEDIUM | spec.md, tasks.md | Glossary quarterly update process underspecified | Detail quarterly update procedures in spec and tasks |
| D7 | Duplication | LOW | tasks.md | Multiple sidebar.js update tasks | Consolidate sidebar updates into single task |
| D8 | Inconsistency | LOW | plan.md, tasks.md | Component file naming convention differs | Standardize component naming convention |

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|
| user-can-upload-assessments | Yes | T020-T026 | Well covered |
| cloud-lab-architecture | Partial | T030-T036 | Only 3/7 tasks completed |
| hardware-requirements-doc | Partial | T040-T046 | Only 3/7 tasks completed |
| glossary-terms-access | Partial | T050-T060 | Only 3/11 tasks completed |
| appendices-reference | Partial | T065-T073 | Only 3/9 tasks completed |
| performance-metrics | No | - | Missing from tasks |
| accessibility-compliance | Partial | T080 | Single task for all WCAG requirements |

**Constitution Alignment Issues:**
- The constitution requires "Test-First (NON-NEGOTIABLE)" approach with TDD, but there are no explicit testing tasks in the tasks.md file beyond manual review.
- The constitution requires "Accessibility and Inclusion" with WCAG 2.1 AA guidelines, but only one task (T080) addresses this across all components.

**Unmapped Tasks:**
- T015 (cross-referencing system) - not directly mapped to a specific requirement
- T081-T082 (search functionality, progress tracking) - enhancement features not in original requirements

**Metrics:**
- Total Requirements: 12 functional requirements + 5 success criteria = 17
- Total Tasks: 34 (21 incomplete + 13 complete)
- Coverage % (requirements with >=1 task): ~70% (12/17 requirements have direct task coverage)
- Ambiguity Count: 2
- Duplication Count: 1
- Critical Issues Count: 1

### Next Actions

- CRITICAL issue identified: Constitution's Test-First principle is not reflected in tasks. This must be addressed before implementation.
- HIGH severity issues: Missing performance tasks and accessibility compliance need to be added to the tasks.md file.
- User may proceed with implementation after addressing CRITICAL and HIGH issues.
- Suggested command: Add explicit testing tasks to tasks.md to align with constitution's Test-First principle.

### Remediation

Would you like me to suggest concrete remediation edits for the top 3 issues? (Constitution Test-First compliance, missing performance tasks, and accessibility implementation)

## Outcome

- ✅ Impact: Identified critical constitution alignment issue with Test-First principle
- 🧪 Tests: Analysis of cross-artifact consistency across spec, plan, and tasks
- 📁 Files: Analyzed 4 files (spec.md, plan.md, tasks.md, constitution-full.md)
- 🔁 Next prompts: Remediation suggestions for top issues
- 🧠 Reflection: Analysis revealed gaps between constitution principles and implementation tasks

## Evaluation notes (flywheel)

- Failure modes observed: Constitution principles not properly mapped to implementation tasks
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add explicit testing tasks to align with Test-First principle