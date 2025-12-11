# ADR-002: GitHub Pages Deployment Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 001-docusaurus-init
- **Context:** Need a reliable, cost-effective deployment strategy for the static documentation site that supports automated builds, testing, rollback capabilities, and stays within free tier constraints while meeting performance requirements.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Deploy to GitHub Pages using GitHub Actions with a multi-stage build pipeline optimized for performance and reliability.

- **Hosting:** GitHub Pages (static site hosting)
- **CI/CD:** GitHub Actions with multi-stage workflow
- **Build Strategy:** Matrix testing across Node.js 18/20 with caching
- **Deployment:** Artifact-based deployment with automatic rollbacks
- **Performance:** Multi-layer caching, parallel builds, asset optimization

## Consequences

### Positive

- Zero hosting cost with 100GB/month bandwidth limit
- Tight integration with GitHub repository and workflow
- Automated testing and deployment pipeline
- Excellent preview/deployment environments for PRs
- Built-in SSL certificates and CDN via GitHub's infrastructure
- Artifact-based deployment enables rollbacks and stability
- Matrix testing ensures cross-platform compatibility

### Negative

- Limited to static sites (no server-side capabilities)
- Build time constraints (GitHub Actions timeout limits)
- Vendor lock-in to GitHub ecosystem
- Deployment only triggered on code changes (no external triggers)
- Limited customization compared to dedicated hosting providers
- Potential bandwidth constraints at high traffic volumes

## Alternatives Considered

**Alternative A: Vercel Platform**
- Hosting: Vercel with built-in CI/CD
- Benefits: Faster builds, edge deployments, preview deployments, analytics
- Rejected: Additional cost at scale, vendor lock-in, less integration control

**Alternative B: Netlify with GitHub Actions**
- Hosting: Netlify with external CI/CD
- Benefits: Build plugins, form handling, split testing, edge functions
- Rejected: Cost for advanced features, complexity of split responsibilities

**Alternative C: AWS S3 + CloudFront + CodePipeline**
- Hosting: AWS S3 with CloudFront CDN
- CI/CD: AWS CodePipeline with CodeBuild
- Benefits: Maximum flexibility, enterprise features, global edge network
- Rejected: Significant complexity, cost, operational overhead, over-engineering for static docs

**Alternative D: GitHub Pages + External CI/CD**
- Hosting: GitHub Pages with CircleCI or Travis CI
- Benefits: More powerful CI/CD features, custom environments
- Rejected: Additional complexity, cost, integration overhead, duplication of GitHub's capabilities

## References

- Feature Spec: specs/001-docusaurus-init/spec.md
- Implementation Plan: specs/001-docusaurus-init/plan.md
- Related ADRs: ADR-001 (Frontend Platform Stack)
- Research Evidence: specs/001-docusaurus-init/research.md