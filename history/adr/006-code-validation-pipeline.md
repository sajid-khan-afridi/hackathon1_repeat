# ADR-006: GitHub Actions CI with ros:humble Docker for Code Validation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-12
- **Feature:** 002-mdx-textbook-chapters
- **Context:** Need to ensure all code examples in the robotics textbook execute correctly in a clean ROS 2 Humble environment to maintain learner trust and prevent broken examples from reaching production.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement automated code validation using GitHub Actions CI with Docker containers.

- **CI Platform**: GitHub Actions (native to repository)
- **Container**: Official ROS 2 Humble Docker image (ros:humble-ros-base-jammy)
- **Extraction**: Parse MDX files for code blocks marked with `# test: true`
- **Languages**: Support both Python (rclpy) and C++ (rclcpp) validation
- **Reporting**: Fail PR if any code example doesn't execute successfully
- **Performance**: Parallel execution of code examples with caching for Docker layers

## Consequences

### Positive

- Automated validation prevents broken code from reaching learners
- Consistent testing environment across all contributors
- Early detection of ROS 2 API changes or breaking updates
- Builds contributor confidence - code is tested before merge
- Enables safe refactoring of existing code examples
- Integration with PR workflow provides immediate feedback

### Negative

- Increased CI execution time (mitigated by parallel execution)
- Dependency on Docker infrastructure and ROS 2 Docker image maintenance
- Potential false positives if test environment differs from learner setup
- Additional complexity in CI/CD pipeline maintenance
- Requires careful handling of simulation vs hardware-specific code

## Alternatives Considered

**Alternative A: Manual Code Testing**
- Approach: Human reviewers manually test each code example
- Benefits: No CI infrastructure needed, human intuition catches edge cases
- Rejected: Error-prone, inconsistent, doesn't scale, slows down review process

**Alternative B: No Validation Pipeline**
- Approach: Trust authors to test their code examples
- Benefits: Fastest development cycle, no infrastructure cost
- Rejected: High risk of broken examples reaching production, damages learner experience

**Alternative C: Linting and Syntax Checking Only**
- Approach: Use linters to check syntax without execution
- Benefits: Fast feedback, catches many errors
- Rejected: Doesn't catch runtime errors, missing dependencies, or API mismatches

**Alternative D: Custom ROS 2 Build Environment**
- Approach: Build custom ROS 2 workspace with specific packages
- Benefits: Tailored to project needs
- Rejected: Complex maintenance, builds would be slow, standard Docker image is sufficient

## References

- Feature Spec: specs/002-mdx-textbook-chapters/spec.md
- Implementation Plan: specs/002-mdx-textbook-chapters/plan.md
- Related ADRs: ADR-002 (GitHub Pages Deployment)
- CI Configuration: .github/workflows/validate-code-examples.yml