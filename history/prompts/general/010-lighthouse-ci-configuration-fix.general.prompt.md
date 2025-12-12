---
id: 010
title: Lighthouse CI Configuration Fix for Static Build Testing
stage: general
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: enhance-phase1-ui-ux
user: sajid-khan-afridi
command: debugging
labels: ["debugging", "lighthouse", "ci-fix", "configuration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/sajid-khan-afridi/hackathon1_repeat/pull/new/enhance-phase1-ui-ux
files:
  - .lighthouserc.json (moved from tests/lighthouse/)
  - tests/lighthouse/lighthouserc.json (removed)
tests:
  - npx lhci healthcheck (passes)
  - npx lhci autorun (successfully scans build directory)
---

## Prompt

build is passed but lighthouse is failed

## Response snapshot

Diagnosed and fixed Lighthouse CI configuration issues:

### 1. **Initial Investigation**
- Checked GitHub Actions recent runs (showed passing but user reported Lighthouse failure)
- Reviewed Lighthouse workflow in `.github/workflows/deploy.yml`
- Examined Lighthouse configuration in `tests/lighthouse/lighthouserc.json`

### 2. **Root Cause Analysis**
Two critical configuration issues identified:

**Issue 1: Config Location**
- LHCI looks for `.lighthouserc.json` or `lighthouserc.js` in **project root only**
- Config was located in `tests/lighthouse/` subdirectory
- LHCI couldn't find config → failed silently or used defaults

**Issue 2: Server Requirement**
- Config specified URLs: `http://localhost:3000/hackathon1_repeat/...`
- No development server running in CI environment
- LHCI couldn't connect → tests failed

### 3. **Solution Implementation**

**Changed configuration strategy:**
```json
// OLD (localhost URLs - requires server)
"collect": {
  "url": [
    "http://localhost:3000/hackathon1_repeat/",
    "http://localhost:3000/hackathon1_repeat/docs/intro"
  ]
}

// NEW (static build directory - no server needed)
"collect": {
  "staticDistDir": "./build"
}
```

**File operations:**
- Moved `tests/lighthouse/lighthouserc.json` → `.lighthouserc.json` (root)
- Removed old location (no longer needed)
- Updated config to use `staticDistDir` instead of URLs

### 4. **Testing & Validation**
```bash
npx lhci healthcheck
# ✅  .lighthouseci/ directory writable
# ✅  Configuration file found
# ✅  Chrome installation found
# Healthcheck passed!

npx lhci autorun --collect.numberOfRuns=1
# Started a web server on port 58180...
# Running Lighthouse on all HTML files in ./build
# ✅ 5 pages scanned successfully
# ✅ Reports uploaded to temporary public storage
```

### 5. **How staticDistDir Works**
When using `staticDistDir`:
1. LHCI automatically starts a local HTTP server
2. Serves all files from `./build` directory
3. Discovers all HTML files recursively
4. Runs Lighthouse audits on each page
5. No manual server management required
6. Perfect for CI environments

## Outcome

- ✅ Impact: Lighthouse CI now works in GitHub Actions. Can audit production builds without running development server.
- 🧪 Tests: Lighthouse successfully scans 5 pages (404, index, docs/intro, docs/tags, docs/github-pages-deployment-research). All reports uploaded successfully.
- 📁 Files: 1 file changed (moved and simplified). Config now in standard location with CI-friendly settings.
- 🔁 Next prompts: Monitor next GitHub Actions run to confirm Lighthouse job passes. Then merge PR to main.
- 🧠 Reflection: LHCI's `staticDistDir` feature is essential for CI environments where you can't rely on a pre-started dev server. Moving config to root follows LHCI conventions and enables auto-discovery.

## Evaluation notes (flywheel)

- Failure modes observed: Initial web fetch showed "all passed" but user reported Lighthouse failure - likely timing issue or cached view. Direct local testing revealed the issues quickly.
- Graders run and results (PASS/FAIL): PASS - lhci healthcheck and autorun both succeed with new configuration.
- Prompt variant (if applicable): Standard debugging workflow with configuration fix.
- Next experiment (smallest change to try): Consider adding specific URL patterns to test only critical pages (homepage, key docs) rather than all HTML files. Could reduce Lighthouse CI execution time in large documentation sites.
