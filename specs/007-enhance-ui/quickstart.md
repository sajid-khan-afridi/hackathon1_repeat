# Phase 1: Quickstart - UI Enhancement Development

**Feature Branch**: `007-enhance-ui`
**Date**: 2025-12-27
**Status**: Complete

## Prerequisites

- Node.js >= 20.0
- npm >= 10.0
- Git
- VS Code (recommended) with ESLint and Prettier extensions

## Quick Setup

### 1. Clone and Install

```bash
# Clone repository
git clone https://github.com/your-org/hackathon1_repeat.git
cd hackathon1_repeat

# Switch to feature branch
git checkout 007-enhance-ui

# Install dependencies
npm install
```

### 2. Start Development Server

```bash
# Start Docusaurus dev server
npm start

# Opens at http://localhost:3000
```

### 3. Verify Setup

Open browser to:
- Home page: http://localhost:3000
- Chatbot test: http://localhost:3000/chatbot
- Components: http://localhost:3000/docs/intro

---

## Development Workflow

### Running Tests

```bash
# Unit and component tests
npm test

# Watch mode for TDD
npm run test:watch

# Coverage report
npm run test:coverage

# E2E tests (requires build)
npm run build
npm run test:e2e

# E2E with UI
npm run test:e2e:ui
```

### Code Quality

```bash
# Lint code
npm run lint

# Fix lint issues
npm run lint:fix

# Format code
npm run format

# Type check
npm run typecheck

# Full CI check
npm run ci
```

### Building

```bash
# Production build
npm run build

# Serve production build locally
npm run serve
```

---

## Key Directories for Phase 6

### Components to Enhance

```
src/components/
├── ChatPanel/           # NEW: Create docked chat panel
├── animations/          # NEW: Animation utilities
├── HomepageFeatures/    # ENHANCE: Add scroll reveal
├── FloatingChatPopup/   # REFACTOR: Extract to ChatPanel
├── SkeletonLoader/      # NEW: Loading states
└── Navbar/              # ENHANCE: Icons and tooltips
```

### Pages to Enhance

```
src/pages/
└── index.tsx            # ENHANCE: Hero animation
```

### Test Files

```
tests/
├── e2e/                 # Playwright E2E tests
├── unit/                # Jest unit tests
└── component/           # React Testing Library
```

---

## Animation Development Guide

### 1. CSS-First Approach

Always try CSS animations first:

```css
/* In ComponentName.module.css */
.fadeIn {
  animation: fadeIn 300ms ease-out forwards;
}

@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Always include reduced motion override */
@media (prefers-reduced-motion: reduce) {
  .fadeIn {
    animation: none;
    opacity: 1;
    transform: none;
  }
}
```

### 2. Using Framer Motion (When CSS Is Insufficient)

```typescript
// Only for complex animations
import { motion } from 'framer-motion';

const variants = {
  hidden: { opacity: 0, x: -20 },
  visible: { opacity: 1, x: 0 },
};

<motion.div
  variants={variants}
  initial="hidden"
  animate="visible"
  transition={{ duration: 0.3 }}
/>
```

### 3. Scroll Reveal Pattern

```typescript
import { useScrollReveal } from '@/components/animations';

function Section() {
  const { ref, isVisible } = useScrollReveal({ threshold: 0.2 });

  return (
    <section
      ref={ref}
      className={clsx(styles.section, { [styles.visible]: isVisible })}
    >
      {/* content */}
    </section>
  );
}
```

### 4. Reduced Motion Hook

```typescript
import { useReducedMotion } from '@/components/animations';

function AnimatedComponent() {
  const { prefersReducedMotion, getAdjustedDuration } = useReducedMotion();

  const duration = getAdjustedDuration(300); // Returns 0.01 if reduced motion

  return (
    <div style={{ transitionDuration: `${duration}ms` }}>
      {/* content */}
    </div>
  );
}
```

---

## Testing Animations

### 1. Reduced Motion Testing

```typescript
// In Playwright E2E test
test('respects reduced motion preference', async ({ page }) => {
  await page.emulateMedia({ reducedMotion: 'reduce' });
  await page.goto('/');

  // Verify instant transitions
  const hero = page.locator('[data-testid="hero-title"]');
  await expect(hero).toBeVisible();

  const transform = await hero.evaluate(
    el => window.getComputedStyle(el).transform
  );
  expect(transform).toBe('none');
});
```

### 2. Component Animation Testing

```typescript
// In component test
import { render, screen, waitFor } from '@testing-library/react';

test('shows fade-in animation', async () => {
  render(<FadeIn>Content</FadeIn>);

  const element = screen.getByText('Content');

  // Initially hidden
  expect(element).toHaveClass('fadeIn');

  // Wait for animation
  await waitFor(() => {
    expect(element).toHaveStyle({ opacity: '1' });
  });
});
```

### 3. Performance Testing

```typescript
// Measure animation frame rate
test('maintains 60fps during animation', async ({ page }) => {
  await page.goto('/');

  const metrics = await page.evaluate(() => {
    return new Promise(resolve => {
      const frames: number[] = [];
      let lastTime = performance.now();

      const measure = () => {
        const now = performance.now();
        frames.push(1000 / (now - lastTime));
        lastTime = now;

        if (frames.length < 60) {
          requestAnimationFrame(measure);
        } else {
          const avgFps = frames.reduce((a, b) => a + b) / frames.length;
          resolve(avgFps);
        }
      };

      requestAnimationFrame(measure);
    });
  });

  expect(metrics).toBeGreaterThan(55); // Allow small variance
});
```

---

## Environment Variables

No new environment variables required for Phase 6. All animations are client-side.

Existing variables (for reference):
```
# .env.local (optional, for chatbot)
OPENAI_API_KEY=sk-...
QDRANT_API_KEY=...
```

---

## Lighthouse CI

### Run Locally

```bash
# Build and run Lighthouse
npm run lighthouse

# View report in .lighthouseci/
```

### CI Configuration

See `lighthouserc.js` for configuration. Key targets:
- Performance: > 90
- Accessibility: > 90
- Best Practices: > 90
- SEO: > 90

---

## Common Tasks

### Add New Animation Component

1. Create component in `src/components/animations/`
2. Add CSS module with keyframes
3. Include reduced motion override
4. Export from `index.ts`
5. Add unit tests

### Enhance Existing Component

1. Identify component in `src/components/`
2. Add animation CSS to module
3. Use hooks for state-based animations
4. Test with reduced motion
5. Update E2E tests if user-facing

### Add E2E Journey Test

1. Create `journey-XX-name.spec.ts` in `tests/e2e/`
2. Follow existing journey patterns
3. Include viewport variations
4. Test reduced motion variant
5. Add to CI configuration

---

## Troubleshooting

### Animation Not Playing

1. Check `prefers-reduced-motion` in browser dev tools
2. Verify CSS animation is not overridden
3. Check for `display: none` on parent elements
4. Ensure `animation-fill-mode: forwards` for persistent state

### Performance Issues

1. Check DevTools Performance tab
2. Look for layout thrashing (purple bars)
3. Verify only using `transform` and `opacity`
4. Check for missing `will-change` cleanup

### Test Failures

1. Run with `--debug` flag for screenshots
2. Check viewport size matches test
3. Verify mock data is consistent
4. Check for timing issues with `waitFor`

---

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Framer Motion](https://www.framer.com/motion/)
- [Lucide Icons](https://lucide.dev/icons/)
- [WCAG 2.2 Guidelines](https://www.w3.org/WAI/WCAG22/quickref/)
- [Core Web Vitals](https://web.dev/vitals/)

---

## Ready to Start

```bash
# Verify everything works
npm run ci

# Start development
npm start

# Begin with chatbot panel enhancements
# See specs/007-enhance-ui/tasks.md (after /sp.tasks)
```
