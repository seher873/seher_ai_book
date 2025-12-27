# Implementation Plan: Automated Deployment Pipeline

**Branch**: `009-deployment-pipeline` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)

## Summary

Implement a comprehensive CI/CD pipeline that automatically builds, tests, and deploys the Physical AI Textbook application to production environments. The pipeline will handle frontend deployment to Netlify and backend deployment to Hugging Face Spaces, with pre-merge testing, environment management, rollback capabilities, and monitoring. The system must achieve 95% deployment success rate with sub-10-minute deployment times while maintaining zero-downtime updates.

**Primary Requirements**:
- Automated deployment on push to main branch
- Pre-merge testing for all pull requests
- Separate staging and production environments
- Health checks and automated rollback on failure
- Deployment notifications and metrics tracking

**Technical Approach**:
- GitHub Actions for CI/CD orchestration
- Netlify for frontend static site hosting
- Hugging Face Spaces for backend Docker container hosting
- GitHub Secrets for secure credential management
- Slack/GitHub notifications for deployment status

## Technical Context

**Language/Version**:
- Frontend: Node.js 18+, React 18, Docusaurus 3
- Backend: Python 3.9+, FastAPI
- Infrastructure: GitHub Actions (YAML), Dockerfile, Bash scripts

**Primary Dependencies**:
- GitHub Actions workflows (actions/checkout, actions/setup-node, actions/setup-python)
- Netlify CLI for deployment automation
- Docker for backend containerization
- pytest for backend testing
- Jest for frontend testing
- GitHub API for status checks

**Storage**:
- GitHub Actions artifacts for build caching
- Docker registry (GitHub Container Registry) for image storage
- Netlify CDN for frontend static assets
- Hugging Face Spaces for backend container hosting

**Testing**:
- Frontend: Jest + React Testing Library (unit/integration)
- Backend: pytest + pytest-asyncio (unit/integration)
- E2E: Smoke tests via curl/health checks post-deployment
- Workflow: Validate GitHub Actions YAML syntax

**Target Platform**:
- Frontend: Netlify CDN (global edge network)
- Backend: Hugging Face Spaces (containerized cloud environment)
- CI/CD: GitHub Actions (Ubuntu runners)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Frontend build time: < 5 minutes
- Backend Docker build time: < 8 minutes
- Deployment completion: < 10 minutes end-to-end
- Health check response: < 2 seconds
- Test suite execution: < 5 minutes

**Constraints**:
- GitHub Actions free tier: 2000 minutes/month
- Netlify build minutes: 300 minutes/month (free tier)
- Hugging Face Spaces: Community tier limitations
- No downtime during deployments
- Must maintain backward compatibility during rollback

**Scale/Scope**:
- Support 50-100 deployments per month
- Handle 3-5 concurrent PRs with test runs
- Manage 2 environments (staging + production)
- Store 10 deployment versions for rollback

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Principle I: Content Accuracy** - Deployment pipeline ensures tested code reaches production
✅ **Principle II: Educational Clarity** - No direct impact (infrastructure)
✅ **Principle III: Ethical Responsibility** - Automated deployments reduce human error
✅ **Principle IV: Practical Application** - Enables rapid iteration on educational content
✅ **Principle V: Test-First Development** - Pre-merge testing enforces TDD workflow
✅ **Principle VI: Accessibility & Inclusion** - Fast deployments improve content availability
✅ **Principle VII: System Reliability & Observability** - Health checks, monitoring, rollback capabilities

**Security Requirements**:
✅ Secrets stored in GitHub Secrets and Hugging Face Secrets
✅ No sensitive data in build logs
✅ Docker images scanned for vulnerabilities
✅ HTTPS enforced on all endpoints

**Performance Standards**:
✅ Deployment time < 10 minutes (within constitution guidelines)
✅ Health checks < 2 seconds
✅ Test suite < 5 minutes (enables rapid feedback)

**Git Workflow**:
✅ Conventional commit messages enforced
✅ Branch protection on main branch
✅ Required PR reviews before merge

**No violations detected. Proceeding with implementation.**

## Project Structure

### Documentation (this feature)

```text
specs/009-deployment-pipeline/
├── plan.md              # This file
├── spec.md              # Feature specification ✅
├── research.md          # Phase 0: CI/CD platform research
├── data-model.md        # Phase 1: Deployment state model
├── quickstart.md        # Phase 1: Deployment runbook
├── checklists/
│   └── requirements.md  # Requirements checklist ✅
├── contracts/
│   └── pipeline-api.yaml # Deployment API contracts
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
.github/
├── workflows/
│   ├── ci.yml                    # PR testing workflow (NEW)
│   ├── deploy-frontend.yml       # Frontend deployment (ENHANCE EXISTING deploy.yml)
│   ├── deploy-backend.yml        # Backend deployment (NEW)
│   ├── staging.yml               # Staging environment (NEW)
│   └── rollback.yml              # Manual rollback workflow (NEW)
└── scripts/
    ├── health-check.sh           # Post-deployment verification (NEW)
    ├── notify-deployment.sh      # Send notifications (NEW)
    └── rollback-helper.sh        # Rollback automation (NEW)

backend/
├── Dockerfile                    # Container definition (ENHANCE EXISTING)
├── docker-compose.yml            # Local development (NEW)
├── .dockerignore                 # Build optimization (NEW)
├── main.py                       # FastAPI app (EXISTING)
├── requirements.txt              # Python deps (EXISTING)
├── tests/                        # Test suite (EXISTING)
└── scripts/
    ├── init_db.py                # DB initialization (EXISTING)
    └── migrate.sh                # Migration runner (NEW)

frontend/
├── package.json                  # Build scripts (EXISTING)
├── docusaurus.config.js          # Docusaurus config (EXISTING)
├── babel.config.js               # Babel config (EXISTING)
├── jest.config.js                # Jest config (EXISTING)
├── netlify.toml                  # Netlify configuration (NEW)
├── tests/                        # Test suite (EXISTING)
└── scripts/
    └── build-check.sh            # Pre-build validation (NEW)

docs/
└── deployment/
    ├── runbook.md                # Deployment procedures (NEW)
    ├── rollback-guide.md         # Rollback procedures (NEW)
    └── troubleshooting.md        # Common issues (NEW)
```

**Structure Decision**: Web application structure with separate frontend and backend deployment paths. GitHub Actions workflows orchestrate CI/CD, with Netlify handling frontend static site deployment and Hugging Face Spaces handling backend containerized deployment. Scripts directory under `.github/` contains shared automation utilities.

## Complexity Tracking

**No Constitution violations requiring justification.**

This feature adds infrastructure automation without introducing code complexity. The deployment pipeline is declarative (YAML-based) and follows infrastructure-as-code best practices.

## Phase 0: Research & Validation

### Research Areas

1. **GitHub Actions Optimization**
   - Investigate caching strategies for npm and pip dependencies
   - Research parallel job execution for faster builds
   - Evaluate GitHub Actions marketplace for deployment actions
   - Study rate limits and usage optimization

2. **Netlify Deployment**
   - Review Netlify CLI vs API deployment methods
   - Investigate build plugin ecosystem
   - Research preview deployments for staging
   - Evaluate custom domain and SSL certificate management

3. **Hugging Face Spaces**
   - Study Docker deployment workflow
   - Investigate environment variable management
   - Research health check and startup verification
   - Evaluate resource limits and scaling options

4. **Health Check Strategies**
   - Research best practices for post-deployment verification
   - Investigate retry logic and timeout configurations
   - Evaluate endpoint monitoring tools
   - Study rollback trigger mechanisms

5. **Notification Systems**
   - Compare Slack vs Discord vs email for notifications
   - Research GitHub Actions notification integrations
   - Evaluate deployment metrics tracking
   - Study incident reporting workflows

### Validation Criteria

- [ ] GitHub Actions workflows can access required secrets
- [ ] Netlify API authentication works with service tokens
- [ ] Hugging Face Spaces supports Docker deployments
- [ ] Health check endpoints respond within 2 seconds
- [ ] Rollback can be triggered via GitHub Actions workflow_dispatch
- [ ] Notifications deliver within 30 seconds of events
- [ ] Test suite completes within 5-minute target
- [ ] Docker images build and push successfully

### Deliverable

`research.md` documenting:
- Selected CI/CD patterns and rationale
- Deployment platform capabilities and limitations
- Health check implementation approach
- Notification system integration details
- Risk assessment and mitigation strategies

## Phase 1: Architecture & Design

### 1. Deployment State Model

Design the data structure for tracking deployment status, history, and metrics.

**Key Entities**:
- **Deployment**: ID, version, commit SHA, timestamp, status, environment, duration
- **DeploymentStage**: Stage name (test/build/deploy/verify), status, logs, duration
- **Environment**: Name (staging/production), URLs, config version, active deployment
- **HealthCheck**: Endpoint, status code, response time, timestamp
- **Notification**: Type, recipient, status, timestamp

**Deliverable**: `data-model.md` with entity definitions, state transitions, and retention policies.

### 2. GitHub Actions Workflow Design

Design CI/CD workflows for different scenarios.

**Workflows**:

#### A. PR Testing Workflow (`ci.yml`)
- **Trigger**: Pull request opened/updated
- **Jobs**:
  1. Frontend tests (Jest)
  2. Backend tests (pytest)
  3. Linting (ESLint, Ruff)
  4. Test coverage check (>70%)
  5. Security scan (Dependabot)
- **Gates**: All jobs must pass to allow merge

#### B. Frontend Deployment Workflow (`deploy-frontend.yml`)
- **Trigger**: Push to main branch (path filter: `frontend/**`)
- **Jobs**:
  1. Run tests (reuse from PR)
  2. Build Docusaurus site
  3. Deploy to Netlify production
  4. Run health checks
  5. Send notifications
- **Rollback**: Automatic on health check failure

#### C. Backend Deployment Workflow (`deploy-backend.yml`)
- **Trigger**: Push to main branch (path filter: `backend/**`)
- **Jobs**:
  1. Run tests (reuse from PR)
  2. Build Docker image
  3. Push to GitHub Container Registry
  4. Deploy to Hugging Face Spaces
  5. Run database migrations
  6. Run health checks
  7. Send notifications
- **Rollback**: Manual trigger via workflow_dispatch

#### D. Staging Deployment Workflow (`staging.yml`)
- **Trigger**: Push to staging branch
- **Jobs**: Same as production but deploy to staging environment
- **Purpose**: Pre-production testing

#### E. Rollback Workflow (`rollback.yml`)
- **Trigger**: Manual (workflow_dispatch)
- **Inputs**: Environment, target version
- **Jobs**:
  1. Fetch previous deployment version
  2. Redeploy to specified environment
  3. Run health checks
  4. Send notifications

**Deliverable**: Workflow sequence diagrams and YAML structures in `contracts/workflow-spec.md`.

### 3. Health Check Strategy

Design post-deployment verification system.

**Health Check Endpoints**:
- Frontend: `/` (200 OK, HTML content check)
- Backend: `/health` (200 OK, JSON response with service status)

**Verification Steps**:
1. Wait 30 seconds for deployment propagation
2. Make HTTP request with 10-second timeout
3. Verify status code (200)
4. Verify response content (JSON structure, critical fields)
5. Check dependent services (database, Qdrant, OpenRouter)
6. Retry on failure (max 3 attempts with exponential backoff)

**Failure Actions**:
- Log detailed error information
- Send alert notification
- Trigger automatic rollback (frontend only)
- Create GitHub issue for investigation

**Deliverable**: Health check specification in `contracts/health-check.md`.

### 4. Notification System Design

Design deployment status notification flow.

**Notification Channels**:
- Primary: GitHub commit status checks (visible in PR)
- Secondary: Slack webhook (team channel)
- Fallback: GitHub Actions summary (always available)

**Notification Events**:
- Deployment started
- Tests passed/failed
- Build succeeded/failed
- Deployment completed (success/failure)
- Health check status
- Rollback initiated/completed

**Notification Format**:
```
🚀 Deployment Started
Environment: Production
Commit: 5bf5cd6 - "feat: add new feature"
Author: @seher873
Triggered by: Push to main

✅ Tests Passed (3m 42s)
✅ Build Succeeded (4m 18s)
🔄 Deploying to Netlify...
```

**Deliverable**: Notification templates and webhook configuration in `contracts/notifications.md`.

### 5. Rollback Procedure Design

Design safe and efficient rollback process.

**Rollback Triggers**:
- Manual: Developer initiates via GitHub Actions UI
- Automatic: Health check failure after frontend deployment
- Conditional: Backend requires manual approval due to database migrations

**Rollback Steps**:
1. Identify previous stable deployment version
2. Pull deployment artifacts (Docker image or build artifacts)
3. Redeploy to target environment
4. Run health checks to verify rollback success
5. Update deployment status and notifications
6. Create incident report for post-mortem

**Rollback Constraints**:
- Maximum 10 versions stored for rollback (storage limits)
- Database migrations require manual intervention
- Rollback time target: 3 minutes

**Deliverable**: Rollback procedures and safety checks in `quickstart.md`.

### 6. Environment Configuration

Design environment separation and configuration management.

**Environments**:

| Environment | Branch   | Frontend URL                              | Backend URL                     | Purpose                  |
|-------------|----------|-------------------------------------------|---------------------------------|--------------------------|
| Production  | main     | https://seher-robotic-book.netlify.app    | https://seher873-my-ai-book.hf.space | Live user-facing system  |
| Staging     | staging  | https://staging--seher-robotic-book.netlify.app | https://seher873-staging.hf.space | Pre-production testing   |

**Configuration Management**:
- Secrets: GitHub Secrets (per environment)
- Environment Variables: `.env` files (not committed)
- Feature Flags: Environment-based toggles
- API Keys: Separate keys per environment

**Deliverable**: Environment setup guide in `quickstart.md`.

## Phase 2: Implementation Tasks

Tasks will be generated in `tasks.md` via `/sp.tasks` command.

**High-Level Task Breakdown**:

1. **GitHub Actions Workflows** (5-7 tasks)
   - Create PR testing workflow
   - Enhance frontend deployment workflow
   - Create backend deployment workflow
   - Create staging deployment workflow
   - Create rollback workflow

2. **Deployment Scripts** (4-6 tasks)
   - Implement health check script
   - Implement notification script
   - Implement rollback helper script
   - Add database migration automation

3. **Configuration Files** (3-4 tasks)
   - Create Netlify configuration (netlify.toml)
   - Enhance Dockerfile for production
   - Create docker-compose for local testing
   - Add .dockerignore for build optimization

4. **Documentation** (3-4 tasks)
   - Write deployment runbook
   - Write rollback guide
   - Write troubleshooting guide
   - Update README with deployment info

5. **Testing & Validation** (4-5 tasks)
   - Test PR workflow with sample PR
   - Test frontend deployment end-to-end
   - Test backend deployment end-to-end
   - Test rollback procedure
   - Validate notification delivery

## Key Design Decisions

### 1. GitHub Actions vs Alternatives

**Decision**: Use GitHub Actions for CI/CD orchestration.

**Rationale**:
- Native integration with GitHub repository
- No additional service dependencies
- Free tier sufficient for project scale
- YAML-based declarative configuration
- Large marketplace of pre-built actions

**Alternatives Considered**:
- CircleCI: Additional service to manage, cost implications
- GitLab CI: Would require repository migration
- Jenkins: Self-hosted complexity, maintenance burden

### 2. Netlify for Frontend Hosting

**Decision**: Continue using Netlify for frontend deployment.

**Rationale**:
- Already configured and working
- Excellent DX with automatic previews
- Global CDN for fast content delivery
- Free SSL certificates
- Built-in rollback capabilities

**No Changes Required**: Existing deployment flow is solid.

### 3. Hugging Face Spaces for Backend

**Decision**: Continue using Hugging Face Spaces for backend hosting.

**Rationale**:
- Free tier supports Docker containers
- Simple deployment via Dockerfile
- Automatic HTTPS provisioning
- Community tier suitable for educational project
- Easy integration with ML services (future)

**Enhancement**: Automate deployment via GitHub Actions instead of manual pushes.

### 4. Docker for Backend Packaging

**Decision**: Use Docker for backend containerization.

**Rationale**:
- Ensures consistent environment (dev/staging/prod)
- Simplifies dependency management
- Required by Hugging Face Spaces
- Enables local testing with docker-compose
- Industry-standard deployment method

### 5. Health Check Implementation

**Decision**: Implement custom health check scripts in Bash.

**Rationale**:
- Simple HTTP checks don't require complex tools
- Bash scripts run natively in GitHub Actions
- Easy to debug and customize
- No additional dependencies

**Alternatives Considered**:
- curl + jq: Viable, but custom script provides better error handling
- Third-party monitoring: Overkill for current needs, future enhancement

### 6. Notification Strategy

**Decision**: Use GitHub commit status + Slack webhook.

**Rationale**:
- GitHub status provides visibility in PR interface
- Slack webhook enables real-time team notifications
- Both are free and reliable
- No additional services to manage

**Implementation**: Slack webhook URL stored in GitHub Secrets.

### 7. Rollback Approach

**Decision**: Automatic rollback for frontend, manual for backend.

**Rationale**:
- Frontend is stateless, safe to auto-rollback
- Backend involves database state, requires human judgment
- Reduces risk of data inconsistency
- Aligns with conservative approach for educational platform

### 8. Environment Strategy

**Decision**: Two environments (staging + production), no dev environment.

**Rationale**:
- Developers test locally (sufficient for small team)
- Staging provides pre-production validation
- Production serves end users
- Minimizes infrastructure complexity and cost

## Integration Points

### External Services

1. **GitHub**
   - API: Workflow triggers, status checks
   - Secrets: Credential storage
   - Container Registry: Docker image storage

2. **Netlify**
   - API/CLI: Deployment automation
   - CDN: Content delivery
   - Build hooks: Deployment triggers

3. **Hugging Face Spaces**
   - Dockerfile deployment: Backend hosting
   - Secrets: Environment variable management
   - Logs: Debugging and monitoring

4. **Slack (Optional)**
   - Webhook API: Deployment notifications
   - Message formatting: Rich status updates

### Internal Services

1. **Frontend Build**
   - Input: React/Docusaurus source code
   - Output: Static HTML/CSS/JS in `build/`
   - Tool: npm run build

2. **Backend Tests**
   - Input: Python source code
   - Output: Test results, coverage report
   - Tool: pytest

3. **Frontend Tests**
   - Input: React components
   - Output: Test results, coverage report
   - Tool: Jest

4. **Docker Build**
   - Input: Backend code + requirements.txt
   - Output: Docker image tagged with commit SHA
   - Tool: docker build

## Risk Assessment & Mitigation

### High-Risk Items

1. **Risk**: Deployment failure during peak usage
   - **Impact**: Users unable to access textbook content
   - **Probability**: Low (5%)
   - **Mitigation**: Deploy during off-peak hours (documented in runbook), implement blue-green deployment (future), automatic rollback on failure

2. **Risk**: Secrets exposure in logs or artifacts
   - **Impact**: Security breach, unauthorized API access
   - **Probability**: Low (3%)
   - **Mitigation**: Use GitHub Secrets masking, audit all logging statements, never print environment variables, code review security checklist

3. **Risk**: Database migration failure
   - **Impact**: Backend deployment stuck, data corruption risk
   - **Probability**: Medium (15%)
   - **Mitigation**: Test migrations in staging first, implement migration rollback procedures, backup database before production migrations, manual approval gate

### Medium-Risk Items

4. **Risk**: GitHub Actions quota exhaustion
   - **Impact**: CI/CD pipeline stops working mid-month
   - **Probability**: Low (10%)
   - **Mitigation**: Monitor usage dashboard, optimize workflow caching, parallelize jobs efficiently, upgrade to paid tier if needed ($4/month)

5. **Risk**: Netlify build timeout (15 min limit)
   - **Impact**: Frontend deployment fails
   - **Probability**: Low (5%)
   - **Mitigation**: Optimize build process, cache node_modules, monitor build times, split large bundles

6. **Risk**: Hugging Face Spaces cold start latency
   - **Impact**: Backend unavailable for 30-60 seconds after deployment
   - **Probability**: High (80%)
   - **Mitigation**: Health check retry logic, graceful degradation, consider keep-alive pings (future)

### Low-Risk Items

7. **Risk**: Notification delivery failure
   - **Impact**: Team unaware of deployment status
   - **Probability**: Low (5%)
   - **Mitigation**: Multiple notification channels (GitHub + Slack), fallback to GitHub Actions summary, retry logic on webhook failure

8. **Risk**: Concurrent deployments causing conflicts
   - **Impact**: Inconsistent deployment state
   - **Probability**: Very Low (2%)
   - **Mitigation**: GitHub Actions concurrency control, deployment queue, branch protection rules

## Testing Strategy

### Unit Tests
- Health check script logic
- Notification formatting
- Rollback version selection
- Environment variable validation

### Integration Tests
- GitHub Actions workflow execution (test on feature branch)
- Netlify deployment (staging environment)
- Hugging Face deployment (staging environment)
- Health check endpoint calls
- Notification delivery

### End-to-End Tests
- Complete deployment flow: commit → main → production
- Rollback procedure: production issue → previous version
- Multi-environment: staging → production promotion
- Failure scenarios: test failure → blocked merge, health check failure → rollback

### Manual Tests
- Slack notification appearance and formatting
- GitHub PR status check visibility
- Netlify build logs and diagnostics
- Hugging Face Spaces container startup
- Rollback workflow UI and inputs

## Success Metrics

**Deployment Performance**:
- ✅ Frontend build time < 5 minutes
- ✅ Backend build time < 8 minutes
- ✅ End-to-end deployment < 10 minutes
- ✅ Health check response < 2 seconds

**Reliability**:
- ✅ Deployment success rate > 95%
- ✅ Rollback completion < 3 minutes
- ✅ Zero production downtime during deployments
- ✅ Notification delivery < 30 seconds

**Quality**:
- ✅ 100% of merges have passing tests
- ✅ Test coverage maintained at >70%
- ✅ No secrets exposed in logs or artifacts
- ✅ All deployments tracked with versions

**Developer Experience**:
- ✅ Test feedback within 5 minutes
- ✅ Deployment status visible in PR interface
- ✅ Runbook available for common tasks
- ✅ Troubleshooting guide for failures

## Dependencies & Prerequisites

### Required Access
- [ ] GitHub repository admin access
- [ ] GitHub Actions enabled
- [ ] Netlify account with deployment permissions
- [ ] Netlify API token or CLI authentication
- [ ] Hugging Face account and Spaces access
- [ ] Slack workspace admin (for webhook creation)

### Required Secrets (GitHub Secrets)
- [ ] `NETLIFY_AUTH_TOKEN` - Netlify API authentication
- [ ] `NETLIFY_SITE_ID` - Netlify site identifier
- [ ] `HF_TOKEN` - Hugging Face authentication token
- [ ] `SLACK_WEBHOOK_URL` - Slack incoming webhook URL (optional)
- [ ] `DOCKER_USERNAME` - GitHub Container Registry username
- [ ] `DOCKER_PASSWORD` - GitHub Container Registry token

### Required Configuration
- [ ] Branch protection rules on main branch
- [ ] GitHub Actions enabled in repository settings
- [ ] Netlify site linked to repository
- [ ] Hugging Face Space created for backend
- [ ] Docker registry access configured

### Development Tools
- [ ] Docker Desktop (for local testing)
- [ ] Node.js 18+ (for frontend development)
- [ ] Python 3.9+ (for backend development)
- [ ] GitHub CLI (optional, for testing workflows)

## Timeline Estimate

**Note**: Following constitution guidelines, providing task breakdown without time estimates. Implementation prioritization is left to team planning.

**Phase 0: Research** - Research CI/CD best practices and validate approach
**Phase 1: Design** - Complete architecture and design documentation
**Phase 2: Implementation** - Execute tasks from tasks.md
**Phase 3: Testing** - Validate all workflows and procedures
**Phase 4: Documentation** - Finalize runbooks and guides
**Phase 5: Rollout** - Enable production deployment automation

**Priority Order** (aligns with user story priorities):
1. P1: PR testing workflow (US3)
2. P1: Frontend deployment automation (US1)
3. P1: Backend deployment automation (US2)
4. P2: Staging environment setup (US4)
5. P3: Rollback procedures (US5)
6. P3: Monitoring and notifications (US6)

## Appendix

### Glossary

- **CI/CD**: Continuous Integration / Continuous Deployment
- **Health Check**: Automated verification of service availability
- **Rollback**: Reverting to a previous deployment version
- **Blue-Green Deployment**: Strategy for zero-downtime deployments
- **Workflow**: GitHub Actions automation definition
- **Artifact**: Build output stored for deployment

### References

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Netlify CLI Documentation](https://docs.netlify.com/cli/get-started/)
- [Hugging Face Spaces Docker](https://huggingface.co/docs/hub/spaces-sdks-docker)
- [Docker Best Practices](https://docs.docker.com/develop/dev-best-practices/)
- [Deployment Strategies](https://martinfowler.com/bliki/BlueGreenDeployment.html)

### Related Documentation

- [Constitution](../../.specify/memory/constitution.md) - Project principles
- [README](../../README.md) - Project overview
- [Backend README](../../backend/README.md) - Backend setup guide
- [Frontend README](../../frontend/README.md) - Frontend setup guide
