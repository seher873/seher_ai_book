---
id: 003
title: physical-ai-textbook plan
stage: plan
date: 2025-12-12
surface: agent
model: 
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: 
command: 
labels: [planning, textbook, physical-ai, implementation]
links:
  spec: /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/plan.md
 - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/research.md
 - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/data-model.md
 - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/quickstart.md
 - /mnt/c/Users/user/Desktop/my_ai_book/specs/001-physical-ai-textbook/contracts/api-spec.md
 - /mnt/c/Users/user/Desktop/my_ai_book/history/prompts/physical-ai-textbook/003-physical-ai-textbook-plan.plan.prompt.md
tests:
 - 
---

## Prompt

---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
---

## User Input

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


/sp.plan   Create a full Phase-1 execution plan for building the “Physical AI & Humanoid Robotics” textbook, based entirely on the Constitution and the completed /sp.specify output. Break the project into major phases, milestones, and workstreams: chapter writing, module writing, weekly curriculum expansion, hardware and lab sections, appendices, glossary, and review cycles. For each milestone, define objectives, deliverables, dependencies, and acceptance c

/sp.plan   Create a full Phase-1 execution plan for building the “Physical AI & Humanoid Robotics” textbook, based entirely on the Constitution and the completed /sp.specify output. Break the project into major phases, milestones, and workstreams: chapter writing, module writing, weekly curriculum expansion, hardware and lab sections, appendices, glossary, and review cycles. For each milestone, define objectives, deliverables, dependencies, and acceptance c

## Response snapshot

I have successfully executed the implementation planning workflow for the "Physical AI & Humanoid Robotics" textbook. Here's a summary of the completed work:

1. **Setup**: Successfully ran `.specify/scripts/bash/setup-plan.sh --json` and parsed the JSON output, obtaining FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, and BRANCH information for the 001-physical-ai-textbook feature.

2. **Context Loading**: Loaded the feature specification, constitution, and implementation plan template.

3. **Technical Context**: Filled the Technical Context section in plan.md with appropriate details for a textbook project using Markdown, Docusaurus, Node.js, and Git.

4. **Constitution Check**: Completed the Constitution Check section ensuring compliance with all core principles (Content Accuracy, Educational Clarity, Ethical Responsibility, Practical Application, Continuous Updates, Accessibility and Inclusion).

5. **Phase 0 Research**: Generated a comprehensive research.md document addressing technology stack decisions, content structure, simulation platform comparison, hardware recommendations, VLA model coverage, assessment methods, content verification, ethics integration, accessibility features, and scalability planning.

6. **Phase 1 Design Artifacts**: 
   - Created data-model.md defining content entities like Chapters, Subsections, Modules, Weeks, Labs, Hardware Recommendations, Appendices, Glossary Terms, and Code Examples
   - Created quickstart.md with setup instructions and getting started guide
   - Created contracts/api-spec.md with interface specifications for textbook code examples
   - Created contracts directory

7. **Agent Context Update**: Successfully ran `.specify/scripts/bash/update-agent-context.sh qwen` updating the QWEN.md file with the new project context including language (Markdown, Docusaurus), framework (Node.js), and project type (Documentation/educational content repository).

All required planning artifacts have been generated and the agent context has been updated. The project is ready for the next phase of development.

## Outcome

- ✅ Impact: Comprehensive implementation plan created for Physical AI textbook
- 🧪 Tests: Research and design artifacts thoroughly documented
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, contracts/api-spec.md, and updated agent context
- 🔁 Next prompts: Ready for task breakdown phase with `/sp.tasks`
- 🧠 Reflection: Successfully adapted software planning workflow to educational content project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All planning requirements met
- Prompt variant (if applicable): Standard planning workflow adapted for textbook project
- Next experiment (smallest change to try): Proceed to task breakdown phase