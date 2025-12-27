# Phase 0: Research - UI Enhancement

**Feature Branch**: `007-enhance-ui`
**Date**: 2025-12-27
**Status**: Complete

## Research Tasks

This document resolves all NEEDS CLARIFICATION items from the Technical Context and documents best practices research for each technology choice.

---

## 1. Animation Library Strategy

### Decision: CSS-first with Framer Motion fallback

### Rationale
CSS animations are preferred because:
1. **Zero bundle cost** - Native to browsers, no additional JavaScript
2. **GPU acceleration** - `transform` and `opacity` are compositor-only properties
3. **Reduced motion** - Easy to disable via `@media (prefers-reduced-motion: reduce)`
4. **Performance** - No JavaScript execution during animation frame

Framer Motion is used as fallback when:
1. Complex orchestration is needed (staggered children with dynamic timing)
2. Gesture-based animations (drag-to-resize)
3. Shared layout animations (impossible with pure CSS)

### Alternatives Considered

| Library | Bundle Size | Performance | DX | Decision |
|---------|-------------|-------------|-----|----------|
| CSS animations | 0 KB | Excellent | Good | **Primary** |
| Framer Motion | ~40KB (tree-shakeable to ~15KB) | Good | Excellent | **Fallback** |
| GSAP | ~60KB | Excellent | Good | **Rejected** - too heavy, overkill for UI polish |
| Anime.js | ~17KB | Good | Good | **Rejected** - not React-optimized |
| React Spring | ~25KB | Good | Good | **Rejected** - Framer Motion has better DX |

### Implementation Strategy

```css
/* CSS-first approach */
.fadeIn {
  animation: fadeIn 300ms ease-out forwards;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(10px); }
  to { opacity: 1; transform: translateY(0); }
}

/* Reduced motion override */
@media (prefers-reduced-motion: reduce) {
  .fadeIn {
    animation: none;
    opacity: 1;
    transform: none;
  }
}
```

```typescript
// Framer Motion for complex cases only
import { motion, AnimatePresence } from 'framer-motion';

// Only import motion components when needed
const ChatPanel = motion(Panel);
```

### Bundle Budget Compliance
- CSS animations: 0 KB
- Framer Motion (selective imports): ~12 KB gzipped
- Total: ≤ 15 KB ✅

---

## 2. Icon Library Strategy

### Decision: Lucide React

### Rationale
1. **Tree-shakeable** - Only imports icons you use (~1KB per icon)
2. **Consistent design** - All icons follow same visual language (24x24px, 1.5px stroke)
3. **React-optimized** - Native React components with TypeScript support
4. **Active maintenance** - Regular updates, good community
5. **Constitution alignment** - Explicitly recommended in constitution

### Alternatives Considered

| Library | Tree-shakeable | Style | Bundle/icon | Decision |
|---------|----------------|-------|-------------|----------|
| Lucide React | Yes | Consistent | ~1KB | **Selected** |
| Heroicons | Yes | Heroui style | ~1KB | Good alternative |
| Phosphor | Yes | Multiple weights | ~1.2KB | More complex API |
| React Icons | Partial | Mixed | ~2KB | Inconsistent styles |
| FontAwesome | No | Brand-heavy | Heavy | **Rejected** |

### Usage Pattern

```typescript
import { MessageSquare, Minimize2, X, Search, ChevronDown } from 'lucide-react';

// Consistent sizing
<MessageSquare size={24} strokeWidth={1.5} aria-hidden="true" />

// With accessible label for functional icons
<button aria-label="Close panel">
  <X size={24} />
</button>
```

---

## 3. Chatbot Panel Architecture

### Decision: Docked panel with resize handle

### Research Findings

**Reference Implementations Studied:**
- Zoom chat panel (docked, resizable, collapsible)
- Intercom widget (floating, non-resizable)
- Zendesk widget (modal-based)
- Drift (docked, minimal resize)

### Architecture Decision

```typescript
interface ChatPanelState {
  isOpen: boolean;           // Panel visibility
  isMinimized: boolean;      // Collapsed to bar
  width: number;             // 320-600px
  unreadCount: number;       // Unread messages
  scrollPosition: number;    // For "Jump to latest"
  searchQuery: string;       // Chat search
  searchResults: number[];   // Matching message indices
}

// Persistence strategy
const persistence = {
  isOpen: 'localStorage',      // Survives page refresh
  width: 'localStorage',       // User preference
  isMinimized: 'sessionStorage', // Reset on new session
  unreadCount: 'memory',       // Reset on close
  scrollPosition: 'memory',    // Reset on close
};
```

### RTL Support
```css
/* LTR: dock to right */
.chatPanel {
  inset-inline-end: 0;  /* right in LTR, left in RTL */
  inset-block-start: 64px;
}

/* RTL automatically handled by logical properties */
[dir="rtl"] .chatPanel {
  /* No changes needed - inset-inline-end becomes left */
}
```

### Resize Handle
```typescript
// Resize handle on left edge (right edge in RTL)
const ResizeHandle = () => (
  <div
    className={styles.resizeHandle}
    onMouseDown={handleResizeStart}
    role="separator"
    aria-orientation="vertical"
    aria-label="Resize chat panel"
    tabIndex={0}
  />
);
```

---

## 4. Scroll-Based Animation Strategy

### Decision: IntersectionObserver with CSS animations

### Rationale
1. **Performance** - IntersectionObserver is more efficient than scroll listeners
2. **Battery-friendly** - Doesn't fire on every scroll event
3. **Threshold control** - Precise control over when animations trigger

### Implementation Pattern

```typescript
// useScrollReveal.ts
function useScrollReveal(options: { threshold?: number; once?: boolean } = {}) {
  const { threshold = 0.2, once = true } = options;
  const ref = useRef<HTMLElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    // Respect reduced motion
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
      setIsVisible(true);
      return;
    }

    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          if (once) observer.disconnect();
        }
      },
      { threshold, rootMargin: '0px' }
    );

    observer.observe(element);
    return () => observer.disconnect();
  }, [threshold, once]);

  return { ref, isVisible };
}
```

```css
/* Animation triggered by isVisible class */
.sectionReveal {
  opacity: 0;
  transform: translateY(20px);
  transition: opacity 0.5s ease-out, transform 0.5s ease-out;
}

.sectionReveal.isVisible {
  opacity: 1;
  transform: translateY(0);
}

/* Stagger children */
.sectionReveal.isVisible .child:nth-child(1) { transition-delay: 0ms; }
.sectionReveal.isVisible .child:nth-child(2) { transition-delay: 100ms; }
.sectionReveal.isVisible .child:nth-child(3) { transition-delay: 200ms; }
```

---

## 5. Reduced Motion Implementation

### Decision: CSS media query + React hook

### Implementation

```css
/* Global reduced motion stylesheet */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
    scroll-behavior: auto !important;
  }
}
```

```typescript
// useReducedMotion.ts
function useReducedMotion(): boolean {
  const [prefersReducedMotion, setPrefersReducedMotion] = useState(() =>
    typeof window !== 'undefined'
      ? window.matchMedia('(prefers-reduced-motion: reduce)').matches
      : false
  );

  useEffect(() => {
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    const handler = (e: MediaQueryListEvent) => setPrefersReducedMotion(e.matches);

    mediaQuery.addEventListener('change', handler);
    return () => mediaQuery.removeEventListener('change', handler);
  }, []);

  return prefersReducedMotion;
}
```

### Critical Animations (Always Simplified, Not Removed)
- Loading spinners → Opacity pulse instead of rotation
- Typing indicator → Opacity pulse instead of bouncing dots
- Progress bars → Still animate, but with reduced duration

---

## 6. Performance Optimization Strategy

### Decision: GPU-accelerated properties only

### Allowed Animation Properties (Compositor-only)
1. `transform` - translate, scale, rotate
2. `opacity` - fade in/out

### Forbidden Animation Properties (Trigger layout/paint)
1. `width`, `height` - Use `transform: scale()` instead
2. `top`, `left`, `right`, `bottom` - Use `transform: translate()` instead
3. `margin`, `padding` - Causes layout shift
4. `border-width` - Causes layout shift

### Performance Checklist
- [ ] Use `will-change: transform, opacity` only during animation
- [ ] Remove `will-change` after animation completes
- [ ] No JavaScript animation loops (use CSS or rAF only)
- [ ] Debounce resize handlers to 16ms minimum
- [ ] Limit simultaneous animations to 5 elements

### Monitoring
```typescript
// Frame rate monitoring in development
if (process.env.NODE_ENV === 'development') {
  let lastTime = performance.now();
  const checkFrameRate = () => {
    const now = performance.now();
    const fps = 1000 / (now - lastTime);
    if (fps < 50) console.warn(`Low FPS: ${fps.toFixed(1)}`);
    lastTime = now;
    requestAnimationFrame(checkFrameRate);
  };
  requestAnimationFrame(checkFrameRate);
}
```

---

## 7. Testing Strategy for Animations

### Decision: Visual regression + E2E + Unit tests

### Test Types

| Type | Tool | What to Test |
|------|------|--------------|
| Unit | Jest | Hook logic, state management |
| Component | RTL | Interaction triggers, accessibility |
| Visual Regression | Percy/Chromatic | Animation keyframes, states |
| E2E | Playwright | Full user journeys |
| Performance | Lighthouse CI | Core Web Vitals |

### Playwright Animation Testing

```typescript
// Test animation completion
await page.click('[data-testid="chat-toggle"]');
await expect(page.locator('[data-testid="chat-panel"]')).toBeVisible();
await expect(page.locator('[data-testid="chat-panel"]')).toHaveCSS('transform', 'none');

// Test reduced motion
await page.emulateMedia({ reducedMotion: 'reduce' });
await page.click('[data-testid="chat-toggle"]');
// Should be instant, no transform
```

---

## 8. Existing Component Analysis

### Current State Assessment

| Component | Location | Needs Enhancement |
|-----------|----------|-------------------|
| `EnhancedFloatingChatPopup` | `src/components/FloatingChatPopup/` | Refactor to ChatPanel, add resize, search, unread badge |
| `HomepageFeatures` | `src/components/HomepageFeatures/` | Add scroll reveal, hover lift |
| `ThemeToggle` | `src/components/ThemeToggle/` | Add smooth transition |
| Home page | `src/pages/index.tsx` | Add hero animation, stagger |
| Navbar | Theme default | Swizzle and enhance with icons, tooltips |

### Breaking Changes
None. All enhancements are additive. Existing functionality preserved.

### Migration Path
1. Create new `ChatPanel` component based on `EnhancedFloatingChatPopup`
2. Update `GlobalFloatingChat` to use new `ChatPanel`
3. Preserve existing props and behavior
4. Add new features incrementally

---

## Summary

All research items resolved. Key decisions:

1. **Animation Library**: CSS-first with Framer Motion fallback (≤15KB budget)
2. **Icon Library**: Lucide React (tree-shakeable, constitution-aligned)
3. **Panel Architecture**: Docked with logical CSS properties for RTL
4. **Scroll Animations**: IntersectionObserver (performant, battery-friendly)
5. **Reduced Motion**: CSS media query + React hook
6. **Performance**: GPU-accelerated properties only
7. **Testing**: E2E journeys + visual regression

Ready to proceed with Phase 1: Design & Contracts.
