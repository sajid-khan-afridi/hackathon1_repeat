# Phase 0 Research: Docusaurus Book Infrastructure

**Feature**: 002-docusaurus-book-infra
**Date**: 2025-12-11
**Purpose**: Resolve all "NEEDS CLARIFICATION" items from Technical Context

---

## Research Tasks

### 1. Docusaurus v3 Best Practices for Educational Content

**Question**: What are the recommended Docusaurus v3 configuration patterns for educational textbook platforms with 100+ pages?

**Findings**:

**Decision**: Use Docusaurus v3.1+ with TypeScript configuration
- **Why**: Built-in support for MDX 3, automatic sidebar generation from folder structure, excellent TypeScript DX
- **Performance**: Incremental builds, code splitting by route (automatic), image optimization plugin
- **Scalability**: Proven with 1000+ page sites (React docs, Ionic docs), <5min build times achievable with proper configuration

**Configuration Pattern**:
```typescript
// docusaurus.config.ts
export default {
  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/', // Docs-only mode (no blog)
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/[user]/[repo]/edit/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],
  plugins: [
    '@docusaurus/plugin-ideal-image', // Image optimization
  ],
};
```

**Alternatives Considered**:
- **Next.js + MDX**: More flexible but requires custom routing, sidebar generation, dark mode implementation
- **VitePress**: Excellent performance but Vue-based (team knows React better)
- **GitBook**: SaaS solution, lacks customization for RTL and personalization features in future phases

**Rationale**: Docusaurus provides 80% of required features out-of-box (sidebar, dark mode, responsive, SEO), allowing focus on content quality rather than infrastructure. TypeScript support is first-class.

---

### 2. RTL-Ready CSS Architecture

**Question**: How do we prepare CSS for future Urdu RTL support without implementing full RTL in Phase 1?

**Findings**:

**Decision**: Use CSS Logical Properties instead of directional properties
- **What**: `margin-inline-start` instead of `margin-left`, `padding-block` instead of `padding-top/bottom`
- **Browser Support**: All modern browsers (95%+ global support as of 2024)
- **Migration Effort**: Minimal refactoring when Phase 5 adds RTL (change `dir="ltr"` to `dir="rtl"`)

**Implementation Pattern**:
```css
/* ❌ Old way - hardcoded left/right */
.sidebar {
  margin-left: 20px;
  padding-right: 10px;
}

/* ✅ New way - logical properties */
.sidebar {
  margin-inline-start: 20px;
  padding-inline-end: 10px;
}
```

**Docusaurus Integration**:
- Docusaurus v3 `custom.css` supports logical properties
- Use CSS custom properties for spacing tokens:
  ```css
  :root {
    --spacing-sm: 8px;
    --spacing-md: 16px;
  }
  ```

**Alternatives Considered**:
- **CSS-in-JS with RTL auto-conversion**: Adds build complexity, performance overhead
- **Full RTL implementation now**: Violates YAGNI principle, no Urdu content yet
- **Ignore RTL until Phase 5**: Would require extensive CSS refactoring, breaking changes

**Rationale**: Logical properties are modern CSS best practice regardless of RTL. No performance cost, future-proof architecture.

---

### 3. GitHub Pages Deployment Strategy

**Question**: Should we use gh-pages branch or GitHub Actions with main branch deployment?

**Findings**:

**Decision**: GitHub Actions workflow deploying from main branch
- **Why**: Spec explicitly requires this approach (FR-002), better CI/CD integration
- **Process**: Push to main → Actions builds Docusaurus → Deploys static files to GitHub Pages
- **Build Time**: Typical Docusaurus build for 100 pages: 2-3 minutes (within 5min budget)

**Workflow Configuration**:
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v2
        with:
          path: ./build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    steps:
      - name: Deploy to GitHub Pages
        uses: actions/deploy-pages@v2
```

**Alternatives Considered**:
- **gh-pages branch**: Docusaurus default, but spec requires main-based deployment
- **Manual deployment**: Error-prone, no CI validation
- **Third-party hosting (Netlify/Vercel)**: Exceeds free tier for future phases with backend

**Rationale**: GitHub Actions provides free CI/CD minutes (2000/month), integrates with Lighthouse CI for quality gates, and keeps entire stack on GitHub ecosystem.

---

### 4. Dark Mode Implementation

**Question**: What is the best approach for dark mode with system preference detection and manual toggle persistence?

**Findings**:

**Decision**: Use Docusaurus built-in dark mode with custom toggle component
- **Default Behavior**: Detects `prefers-color-scheme` media query on first visit
- **Persistence**: Stores user choice in `localStorage` key `theme`
- **Toggle Component**: Docusaurus provides `<ThemeToggle>` component out-of-box

**Customization Needed**:
```typescript
// docusaurus.config.ts
export default {
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true, // Auto-detect system preference
    },
  },
};
```

**WCAG AA Contrast Ratios**:
- Light mode: Use Docusaurus default (meets 4.5:1 for text)
- Dark mode: Customize via CSS custom properties
  ```css
  [data-theme='dark'] {
    --ifm-color-primary: #25c2a0;
    --ifm-background-color: #1b1b1d;
    --ifm-font-color-base: #e3e3e3;
  }
  ```

**Testing**:
- Manual: Toggle dark mode, navigate pages, refresh browser
- Automated: Lighthouse CI checks contrast ratios in both modes

**Alternatives Considered**:
- **Custom implementation**: Reinventing wheel, high effort for same result
- **CSS-only (no JS)**: Cannot persist preference across sessions
- **Third-party library**: Unnecessary dependency, Docusaurus covers all requirements

**Rationale**: Docusaurus dark mode is production-ready, WCAG-compliant, and zero-config for basic use. Customization only needed for brand colors.

---

### 5. Responsive Design Patterns

**Question**: What breakpoints and patterns ensure mobile (320px), tablet (768px), and desktop (1024px) responsiveness?

**Findings**:

**Decision**: Use Docusaurus default responsive breakpoints with mobile-first approach
- **Breakpoints** (from `@docusaurus/theme-classic`):
  - Mobile: `320px - 996px`
  - Desktop: `997px+`
- **Sidebar Behavior**:
  - Mobile: Collapsible hamburger menu (auto-enabled by Docusaurus)
  - Desktop: Persistent sidebar (25% width)

**Touch Target Sizes**:
- Docusaurus default buttons/links: `44x44px` minimum (meets FR-007)
- Verify with Lighthouse accessibility audit

**Font Scaling**:
```css
/* Base font size: 16px (readable without zoom) */
html {
  font-size: 16px;
}

@media (max-width: 996px) {
  html {
    font-size: 15px; /* Slightly smaller on mobile for more content */
  }
}
```

**Testing Strategy**:
- Chrome DevTools responsive mode (320px, 768px, 1024px, 1920px)
- Real devices: iPhone SE (320px), iPad (768px), desktop (1920px)
- Playwright E2E tests at each breakpoint

**Alternatives Considered**:
- **Custom breakpoints (320/768/1024)**: Spec mentions these, but Docusaurus default (320/997) covers the range
- **CSS Grid layout**: Docusaurus uses Flexbox (simpler, well-tested)
- **Separate mobile/desktop templates**: Over-engineering, responsive CSS sufficient

**Rationale**: Docusaurus responsive design is battle-tested (thousands of sites), meets all spec requirements without custom code.

---

### 6. Code Syntax Highlighting

**Question**: How do we enable syntax highlighting for Python, C++, ROS 2, and YAML?

**Findings**:

**Decision**: Use Prism.js (Docusaurus default) with language extensions
- **Included by Default**: JavaScript, TypeScript, JSON, Markdown
- **Add via config**: Python, C++, YAML
- **ROS 2**: Use Python syntax (ROS 2 code is Python/C++ based)

**Configuration**:
```typescript
// docusaurus.config.ts
export default {
  themeConfig: {
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: ['python', 'cpp', 'yaml'],
    },
  },
};
```

**Usage in MDX**:
````mdx
```python
# ROS 2 Python publisher example
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```
````

**Performance**: Prism.js is lazy-loaded per language (no impact on pages without code)

**Alternatives Considered**:
- **Highlight.js**: Larger bundle size, less Docusaurus integration
- **Shiki**: Better syntax accuracy but slower build times
- **Custom syntax for ROS 2**: Unnecessary, ROS 2 code is Python/C++

**Rationale**: Prism.js is Docusaurus default, zero-config for common languages, excellent performance.

---

### 7. Performance Optimization Strategy

**Question**: What optimizations ensure Lighthouse Performance > 85 and build time < 5 minutes?

**Findings**:

**Decision**: Enable Docusaurus built-in optimizations + Lighthouse CI
- **Image Optimization**: `@docusaurus/plugin-ideal-image` (lazy loading, responsive images)
- **Code Splitting**: Automatic by Docusaurus (per route)
- **Minification**: Enabled in production build (webpack default)
- **Lighthouse CI**: Run on every PR to prevent regressions

**Build Time Optimizations**:
- **Parallel builds**: Webpack defaults (uses all CPU cores)
- **Incremental builds**: Docusaurus `--incremental` flag (only rebuild changed files)
- **Caching**: GitHub Actions caches `node_modules` and `.docusaurus` folder

**Lighthouse CI Configuration**:
```json
// tests/lighthouse/lighthouserc.json
{
  "ci": {
    "collect": {
      "numberOfRuns": 3,
      "url": ["http://localhost:3000/"]
    },
    "assert": {
      "preset": "lighthouse:recommended",
      "assertions": {
        "categories:performance": ["error", {"minScore": 0.85}],
        "categories:accessibility": ["error", {"minScore": 0.90}],
        "categories:best-practices": ["error", {"minScore": 0.90}]
      }
    }
  }
}
```

**Advanced Optimizations (defer to Phase 6)**:
- Font subsetting (reduces font file size by 50-70%)
- Custom lazy loading for images above fold
- Resource hints (`<link rel="preload">`)

**Alternatives Considered**:
- **Custom webpack config**: High complexity, Docusaurus defaults sufficient
- **Image CDN**: Unnecessary for static site, GitHub Pages is fast enough
- **Server-side rendering**: Not possible with GitHub Pages static hosting

**Rationale**: Docusaurus optimizations achieve 85+ Lighthouse scores out-of-box. Defer advanced optimizations to Phase 6 per spec (FR-023).

---

## Summary

All Technical Context "NEEDS CLARIFICATION" items resolved:

| Item | Decision | Rationale |
|------|----------|-----------|
| **Framework** | Docusaurus v3.1+ | Educational content best practices, TypeScript support, 80% features built-in |
| **RTL Architecture** | CSS Logical Properties | Future-proof, zero performance cost, modern best practice |
| **Deployment** | GitHub Actions from main | Spec requirement (FR-002), CI/CD integration, free tier |
| **Dark Mode** | Docusaurus built-in | WCAG-compliant, system preference + manual toggle, localStorage persistence |
| **Responsive** | Docusaurus defaults (320px/997px) | Covers spec requirements, mobile-first, 44px touch targets |
| **Syntax Highlighting** | Prism.js + Python/C++/YAML | Docusaurus default, lazy-loaded, covers ROS 2 |
| **Performance** | Built-in optimizations + Lighthouse CI | Achieves >85 scores, <5min builds, defer advanced to Phase 6 |

**Next Phase**: Proceed to Phase 1 - Design (data-model.md, contracts/, quickstart.md)
