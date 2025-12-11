# ADR-004: Static Site Generator Selection for Educational Platform

> **Scope**: Choose foundational static site generator technology stack for Physical AI & Humanoid Robotics Textbook Platform, affecting all future phases, content authoring workflow, and deployment architecture.

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 002-docusaurus-book-infra
- **Context:** Phase 1 infrastructure selection for educational content platform

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence affecting all future phases, content authoring DX, deployment strategy
     2) Alternatives: YES - Multiple viable options considered with significant tradeoffs (Next.js, VitePress, Docusaurus)
     3) Scope: YES - Cross-cutting concern influencing content structure, component development, CI/CD, and future feature implementation
-->

## Decision

**Select Docusaurus v3.1+ as the static site generator for the Physical AI & Humanoid Robotics Textbook Platform**

Technology Stack:
- **Static Site Generator**: Docusaurus v3.1+ (TypeScript-first configuration)
- **Content Format**: MDX v3 for rich educational content
- **Styling**: CSS Modules + logical properties for RTL readiness
- **Deployment**: GitHub Actions CI/CD to GitHub Pages
- **Build Tool**: Webpack (Docusaurus internal)
- **Package Manager**: npm (with GitHub Actions caching)

## Consequences

### Positive

**Educational Content Excellence**
- **Built-in Sidebar Navigation**: Auto-generated from folder structure, perfect for textbook organization
- **MDX Support**: Seamless integration of React components within educational content
- **Code Syntax Highlighting**: Prism.js with Python, C++, YAML support for robotics examples
- **Search**: Built-in Algolia documentation search (available for future phases)

**Developer Experience**
- **Zero-Config Setup**: 80% of required features work out-of-box (dark mode, responsive, SEO)
- **TypeScript First-Class**: Full TypeScript support for configuration and components
- **Hot Reload**: Instant preview during content authoring
- **Plugin Ecosystem**: Rich plugins for image optimization, sitemaps, and analytics

**Performance & Scalability**
- **Proven at Scale**: Used by React docs, Ionic docs, and thousands of documentation sites
- **Build Optimization**: Automatic code splitting, lazy loading, and image optimization
- **Lighthouse Ready**: Achieves >85 Performance scores with minimal configuration
- **Incremental Builds**: Fast rebuilds during development

**Deployment Simplicity**
- **Static Output**: Perfect for GitHub Pages free tier (100GB/month)
- **GitHub Actions Integration**: Mature deployment workflows
- **No Server Required**: Eliminates hosting complexity and cost

**Content Authoring Workflow**
- **File-Based**: Authors work with familiar Git workflow
- **Preview Local**: `npm run start` for instant preview
- **Version Control**: All content changes tracked in Git
- **Collaboration**: Standard pull request process for content review

### Negative

**Framework Coupling**
- **Docusaurus-Specific APIs**: Custom components must use Docusaurus theme APIs
- **Migration Cost**: Switching generators later would require content refactoring
- **Learning Curve**: Team must learn Docusaurus-specific patterns

**Performance Limitations**
- **Static Only**: No server-side rendering or dynamic content in Phase 1
- **Bundle Size**: Larger than hand-rolled solutions due to built-in features
- **Build Time**: 2-5 minutes for 100+ pages (within spec limits)

**Design Constraints**
- **Theme Limitations**: Customization through CSS custom properties, not complete design freedom
- **Component Structure**: Must work within Docusaurus component hierarchy
- **Routing**: Fixed URL structure based on docs folder organization

**Future Constraints**
- **Backend Integration**: Will require API client implementation for future phases
- **User Authentication**: Must be added as overlay, not built-in
- **Dynamic Features**: Interactive features require custom React components

## Alternatives Considered

### Alternative A: Next.js + MDX
**Stack**: Next.js 14 App Router + MDX + Tailwind CSS + Vercel

**Pros**:
- **Flexibility**: Complete control over routing, styling, and components
- **React Ecosystem**: Full access to Next.js features and middleware
- **Performance**: Excellent performance with App Router
- **Future-Ready**: Easy to add backend features, API routes, authentication

**Cons**:
- **High Setup Cost**: Must build sidebar navigation, search, and dark mode from scratch
- **Content Management**: No built-in docs-specific features
- **Deployment**: Requires Vercel/Netlify (exceeds free tier for future phases)
- **Maintenance**: More code to maintain for same functionality

**Why Rejected**: Would require 2-3 months of additional development to replicate Docusaurus built-in features, delaying Phase 1 delivery. Higher complexity contradicts "Smallest Viable Change" principle.

### Alternative B: VitePress + Vue
**Stack**: VitePress + Vue 3 + TypeScript + Custom Components

**Pros**:
- **Performance**: Excellent Lighthouse scores and build times
- **Developer Experience**: Fast development server and hot reload
- **Bundle Size**: Smaller than Docusaurus
- **Modern Tooling**: Vite-based build system

**Cons**:
- **Vue Ecosystem**: Team knows React better than Vue
- **Smaller Community**: Fewer educational platform examples
- **Plugin Ecosystem**: Less mature than Docusaurus
- **Learning Curve**: Team would need Vue training

**Why Rejected**: Technology stack mismatch with team expertise would slow development. Fewer educational platform examples means more unknowns for implementation.

### Alternative C: GitBook SaaS
**Stack**: GitBook hosted platform + Custom integrations

**Pros**:
- **Zero Maintenance**: No infrastructure to manage
- **Collaboration**: Built-in authoring workflow
- **Search**: Full-text search included
- **Analytics**: Built-in usage analytics

**Cons**:
- **Limited Customization**: Cannot implement RTL preparation or custom components
- **Vendor Lock-in**: Migration would be difficult
- **Cost**: Subscription fees for team usage
- **No GitHub Integration**: Cannot leverage existing Git workflow

**Why Rejected**: Lack of customization options prevents RTL preparation and custom React components for future phases. Would not meet spec requirements for advanced features.

### Alternative D: Custom Static Site Generator
**Stack**: Rollup + React + Custom build scripts

**Pros**:
- **Maximum Control**: Complete control over every aspect
- **Minimal Bundle**: Only include necessary features
- **Learning Opportunity**: Deep understanding of build process

**Cons**:
- **High Development Cost**: Would need to rebuild Docusaurus functionality
- **Maintenance Burden**: Ongoing maintenance of custom build system
- **Slower Delivery**: Significantly delays Phase 1
- **Risk**: Higher chance of bugs and performance issues

**Why Rejected**: Violates "Quality Over Speed" principle by building unnecessarily complex solution. Docusaurus provides mature, tested implementation of required features.

## Implementation Evidence

**Research Findings** (from `specs/002-docusaurus-book-infra/research.md`):

- **Docusaurus v3 Performance**: Proven to handle 1000+ page sites with <5min build times
- **Educational Platform Adoption**: Used by major educational platforms (React docs, Jest docs)
- **TypeScript Integration**: First-class TypeScript support for configuration and components
- **GitHub Pages Deployment**: Mature workflows and community examples
- **RTL Preparation**: CSS logical properties fully supported

**Success Criteria Alignment**:
- ✅ Lighthouse Performance > 85 (achievable with default config)
- ✅ Build time < 5 minutes (verified with 100+ pages)
- ✅ GitHub Pages deployment (native support)
- ✅ MDX content support (built-in)
- ✅ Code syntax highlighting (Prism.js plugin)
- ✅ Dark mode (built-in)
- ✅ Responsive design (mobile-first defaults)

## Risk Mitigations

### Negative Consequence Mitigations

**Framework Coupling**:
- **Mitigation**: Document all Docusaurus-specific patterns in quickstart.md
- **Future Option**: Migration path documented if ever needed

**Performance Limitations**:
- **Mitigation**: Enable Docusaurus optimization plugins (image, lazy loading)
- **Monitoring**: Include Lighthouse CI to catch regressions
- **Future Option**: Advanced optimizations deferred to Phase 6 per spec

**Design Constraints**:
- **Mitigation**: Use CSS custom properties for brand customization
- **Examples**: Create reference implementations for common patterns

### Implementation Risks

**Build Time Growth**:
- **Risk**: Build time may exceed 5 minutes as content grows
- **Mitigation**: Monitor build metrics, use incremental builds
- **Threshold**: Alert at 70% of time budget (3.5 minutes)

**Content Complexity**:
- **Risk**: Complex MDX components may slow builds
- **Mitigation**: Component testing, build performance monitoring
- **Guidelines**: Document performance best practices for authors

## Future Considerations

**Phase 2-6 Compatibility**:
- **Backend Integration**: Docusaurus can consume APIs via client-side fetch
- **Authentication**: Can overlay authentication without changing core
- **Search**: Algolia integration available when needed
- **Analytics**: Google Analytics and custom tracking supported

**Scaling Strategy**:
- **CDN Integration**: GitHub Pages + Cloudflare for global distribution
- **Advanced Features**: Custom React components can add any needed functionality
- **Performance Optimization**: Phase 6 allocated for advanced optimizations

## References

- Feature Spec: `specs/002-docusaurus-book-infra/spec.md`
- Implementation Plan: `specs/002-docusaurus-book-infra/plan.md`
- Research Findings: `specs/002-docusaurus-book-infra/research.md`
- Related ADRs:
  - ADR-001: Frontend Platform Stack
  - ADR-002: GitHub Pages Deployment
  - ADR-003: Quality Gates and Acceptance Criteria Reconciliation
- Constitution: `.specify/memory/constitution.md` (Quality Over Speed principle)
- Task List: `specs/002-docusaurus-book-infra/tasks.md` (implementation tasks based on this decision)

## Acceptance Criteria for This ADR

- [ ] ADR file created at `history/adr/004-static-site-generator-selection.md`
- [ ] Implementation plan references this decision
- [ ] Task list includes Docusaurus-specific implementation tasks
- [ ] PHR created documenting this ADR work
- [ ] Quickstart guide includes Docusaurus-specific setup instructions
- [ ] Lighthouse CI configured to validate performance assumptions
- [ ] RTL preparation implemented using CSS logical properties (per research findings)