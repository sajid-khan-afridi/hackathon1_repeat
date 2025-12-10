---
id: 002
title: git workflow commit and PR
stage: general
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: none
branch: feature/update-project-constitution-v1
user: Sajid Khan Afridi
command: /sp.git.commit_pr
labels: ["git", "workflow", "pr", "branch"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - 1.md
  - 2.md
  - history/prompts/constitution/001-establish-project-constitution-v1-0-0-creation.constitution.prompt.md
  - history/prompts/general/002-git-workflow-commit-pr.general.prompt.md
tests:
  -
---

## Prompt

/sp.git.commit_pr

Your task is to intelligently executes git workflows to commit the work and create PR following your Principles

# Agentic Git Workflow Agent

## Core Principle

You are an autonomous Git agent. Your job is to **fulfill the user's intent efficiently**. You have agency to:
- Analyze the current state independently
- Make intelligent decisions about the best workflow
- Execute steps without asking permission for each one
- Invoke the human validator only when the decision requires their judgment

The human is not a step-orchestrator. The human is an **intent-provider** and **decision validator**.

## Response snapshot

Analyzed repository state and found clean working tree on feature branch 'feature/update-project-constitution-v1'. Branch already has commit with comprehensive project constitution changes and is pushed to remote. No additional changes to commit. Provided manual PR creation instructions since GitHub CLI not available.

## Outcome

- ✅ Impact: workflow analyzed, no uncommitted changes found
- 🧪 Tests: N/A - no test execution required
- 📁 Files: PHR created at history/prompts/general/002-git-workflow-commit-pr.general.prompt.md
- 🔁 Next prompts: Manual PR creation
- 🧠 Reflection: Repository already in committed state with branch pushed to remote

## Evaluation notes (flywheel)

- Failure modes observed: GitHub CLI not available for automatic PR creation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add fallback PR creation via curl to GitHub API
