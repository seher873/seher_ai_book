---
id: 002
title: Impl Plan
stage: plan
date: 2025-12-16
surface: agent
model: Qwen
feature: 001-finalize-textbook
branch: 001-finalize-textbook
user: user
command: $ARGUMENTS
labels: [plan, implementation, textbook]
links:
  spec: /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/plan.md
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/research.md
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/data-model.md
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/quickstart.md
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/contracts/textbook-assessment-api.yaml
  - /mnt/c/Users/user/Desktop/my_ai_book/QWEN.md
tests:
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/plan.md
  - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/research.md
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/bash/update-agent-context.sh qwen`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

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

I've successfully completed the implementation planning workflow for the "Finalize textbook Phase-1" feature. Here's what was accomplished:

### Phase 0: Outline & Research
- **Technical Context** was filled with appropriate details about the technology stack (Markdown/CommonMark, Docusaurus v3.x, Node.js v18+, React)
- **Constitution Check** was completed ensuring compliance with all core principles
- **research.md** was created resolving all technical clarifications about assessment formats, lab architecture (cloud-based Jupyter notebooks), hardware requirements documentation, glossary maintenance strategy, and appendix organization

### Phase 1: Design & Contracts
- **data-model.md** was created defining the core entities: AssessmentQuestion, LabExercise, HardwareProfile, GlossaryTerm, and Appendix
- **API contracts** were defined in the `/contracts` directory with an OpenAPI specification for the textbook components
- **quickstart.md** was created providing a step-by-step guide for implementing the new textbook features
- **Agent context** was updated by running the update-agent-context.sh script, adding new technology information to the Qwen Code context

### Artifacts Generated:
1. Updated `plan.md` with complete technical context and constitution check
2. Created `research.md` with comprehensive research findings
3. Created `data-model.md` with detailed entity definitions
4. Created `contracts/textbook-assessment-api.yaml` with API specification
5. Created `quickstart.md` with implementation guide
6. Updated agent context in `QWEN.md`

The branch used was `001-finalize-textbook`, and all artifacts are located in the `/mnt/c/Users/user/Desktop/my_ai_book/specs/001-finalize-textbook/` directory.

Note: There was a warning about multiple spec directories with prefix '001', but this didn't prevent the successful completion of the planning workflow.

## Outcome

- ✅ Impact: Complete implementation plan for textbook assessments, lab architecture, hardware alignment, glossary, and appendices
- 🧪 Tests: N/A (Planning phase, no tests to run yet)
- 📁 Files: 6 new files created, 1 updated
- 🔁 Next prompts: Implementation tasks based on the plan
- 🧠 Reflection: Planning phase complete, ready for task breakdown and implementation

## Evaluation notes (flywheel)

- Failure modes observed: None significant (minor warning about multiple spec directories)
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin task breakdown based on the implementation plan