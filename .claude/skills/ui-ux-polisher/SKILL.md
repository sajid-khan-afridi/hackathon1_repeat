---
name: ui-ux-polisher
description: >
  UI/UX polish and enhancement skill. Use when user requests: menu alignment, navbar/side menu polish, responsiveness fixes, mobile layout issues, typography improvements, visual hierarchy, submenu animations, professional UI look, design consistency, or accessibility improvements. Guides systematic UI auditing, diagnosis, and implementation of tasteful enhancements with animations.
allowed-tools:
  - Read
  - Grep
  - Glob
  - Edit
  - Write
  - Bash
---

# UI/UX Polisher Skill

**Purpose:** Systematically audit, diagnose, and enhance UI/UX quality with professional polish, consistent design patterns, smooth animations, and accessibility compliance.

## When to Use This Skill

Invoke this skill when the user mentions:
- "UI/UX enhance", "professional look", "polish the UI"
- "menu alignment", "navbar polish", "side menu issues"
- "responsiveness", "mobile layout", "breakpoint issues"
- "add animation", "smooth transitions", "menu animations"
- "submenu expand/collapse", "dropdown animations"
- "typography", "visual hierarchy", "spacing issues"
- "accessibility", "keyboard navigation", "focus management"

## Quick Start Workflow

When invoked, Claude should:

1. **Audit Phase** (5 min)
   - Locate UI components: `components/`, `app/`, `pages/`, `layouts/`
   - Find navigation: navbar, side menu, drawer components
   - Identify styling approach (Tailwind, CSS Modules, SCSS, styled-components)
   - Check for animation library (Framer Motion, React Spring, CSS-only)
   - Read existing design tokens/theme files

2. **Diagnosis Phase** (5 min)
   - Run the project locally (if not running) to inspect UI
   - Document issues found:
     - Alignment problems (menu items, buttons, icons)
     - Spacing inconsistencies (padding, margins, gaps)
     - Typography issues (font scale, line-height, hierarchy)
     - Responsive breakpoints (overflow, mobile menu)
     - Animation gaps (abrupt state changes)
     - Accessibility issues (focus rings, keyboard nav, ARIA)

3. **Planning Phase** (2 min)
   - Group related fixes into clusters:
     - Cluster 1: Menu alignment + spacing
     - Cluster 2: Responsive layout
     - Cluster 3: Animations (menu, submenu)
     - Cluster 4: Accessibility fixes
   - Propose implementation order (smallest safe changes first)

4. **Implementation Phase** (iterative)
   - Make **one cluster of changes at a time**
   - Prefer minimal diffs (edit existing code, don't rewrite)
   - Follow existing patterns in codebase
   - Add animations with `prefers-reduced-motion` support
   - Preserve all functionality and routing

5. **Verification Phase** (per cluster)
   - Run lint/typecheck: `npm run lint`, `npm run type-check`
   - Build check: `npm run build`
   - Manual tests: responsive views, keyboard navigation
   - Verify reduced-motion fallback works

## Core UI/UX Principles

### Menu & Submenu Animation

**Goal:** Smooth, premium feel without jarring motion.

- **Duration:** 150–250ms (fast enough to feel instant, slow enough to perceive)
- **Easing:** `cubic-bezier(0.4, 0, 0.2, 1)` or `ease-out` (no bounce unless brand requires it)
- **Properties to animate:**
  - `height` or `max-height` (for expand/collapse)
  - `opacity` (for fade in/out)
  - `transform: translateY()` (for slide effects)
- **Reduced motion:** Always include `@media (prefers-reduced-motion: reduce)` with instant transitions

**Example patterns:**
- Framer Motion: `initial`, `animate`, `exit` with `AnimatePresence`
- CSS: `transition: all 200ms ease-out` with conditional classes
- Submenu: parent sets `aria-expanded`, children animate based on state

### Responsive & Layout

**Breakpoints** (Tailwind defaults, adjust if different):
- Mobile: `< 640px` (sm)
- Tablet: `640px – 1024px` (md/lg)
- Desktop: `≥ 1024px` (xl)

**Common fixes:**
- **Horizontal overflow:** Add `overflow-x-hidden` to containers, ensure content respects viewport
- **Menu on mobile:** Hamburger menu → slide-in drawer or dropdown
- **Touch targets:** Minimum 44×44px (iOS), prefer 48×48px (Material)
- **Spacing system:** Use consistent scale (4px, 8px, 16px, 24px, 32px...)
- **Flexbox/Grid:** Prefer modern layouts over floats/absolute positioning

### Typography & Visual Hierarchy

**Font Scale** (adjust to project):
- Hero/H1: 2.5rem – 4rem
- H2: 2rem – 3rem
- H3: 1.5rem – 2rem
- Body: 1rem (16px base)
- Small: 0.875rem

**Line Height:**
- Headings: 1.2 – 1.3
- Body: 1.5 – 1.7
- Avoid line-height < 1.2 or > 2

**Contrast:**
- Text on background: ≥ 4.5:1 (WCAG AA)
- Large text (≥ 18pt): ≥ 3:1
- Use tools: WebAIM Contrast Checker

**Hierarchy:**
- Use size + weight + spacing to create visual layers
- Avoid more than 3 font weights in one view
- Consistent heading scale throughout app

### Accessibility (A11y)

**Keyboard Navigation:**
- All interactive elements focusable (buttons, links, inputs)
- Logical tab order (follows visual layout)
- Visible focus indicator (outline or custom ring)
- Escape key closes modals/menus
- Enter/Space activates buttons/links

**ARIA for Menus:**
```html
<button aria-expanded="false" aria-controls="submenu-id">
  Menu Item
</button>
<div id="submenu-id" role="region" aria-labelledby="button-id">
  <!-- Submenu content -->
</div>
```

**Focus Management:**
- When opening menu/modal: move focus to first item or close button
- When closing: return focus to trigger element
- Trap focus inside modals (no Tab escaping)

**Screen Readers:**
- Use semantic HTML (`<nav>`, `<button>`, `<ul>`, `<li>`)
- Avoid `<div>` with `onClick` (use `<button>` instead)
- Provide `aria-label` for icon-only buttons

## Implementation Guidelines

### DO:
- ✅ Start with smallest safe changes (alignment/spacing first)
- ✅ Follow existing code patterns and naming conventions
- ✅ Use project's existing spacing/color tokens if available
- ✅ Add animations gradually (menu first, then submenu, then micro-interactions)
- ✅ Test on multiple viewports (mobile 375px, tablet 768px, desktop 1440px)
- ✅ Preserve all existing functionality and routes
- ✅ Include `prefers-reduced-motion` for all animations
- ✅ Use semantic HTML and proper ARIA attributes

### DON'T:
- ❌ Change brand colors/fonts without explicit user request
- ❌ Rewrite entire components (prefer targeted edits)
- ❌ Add heavy animation libraries unless project already uses them
- ❌ Use fixed pixel widths that break responsiveness
- ❌ Remove existing accessibility features
- ❌ Add flashy/distracting animations (no bounce, no spin, no excessive motion)
- ❌ Break existing routes or navigation behavior

## Output Format

After each implementation cluster, report:

```markdown
### ✅ [Cluster Name] Complete

**Files Changed:**
- `components/Navbar.tsx` (lines 45-67: menu alignment + spacing)
- `components/Sidebar.tsx` (lines 23-45: submenu animation)

**Changes Made:**
1. Fixed menu item alignment using flexbox with consistent gap
2. Added smooth 200ms expand/collapse animation for submenu
3. Included reduced-motion fallback (instant transition)
4. Improved focus ring visibility (2px outline, brand color)

**Verification:**
- ✅ `npm run lint` passed
- ✅ `npm run build` successful
- ✅ Tested on mobile (375px), tablet (768px), desktop (1440px)
- ✅ Keyboard navigation works (Tab, Enter, Escape)
- ✅ Reduced motion respected in browser settings

**Next Steps:**
- Continue with Cluster 2: Responsive layout fixes
```

## Progressive Enhancement Strategy

1. **Foundation First:** Fix layout, spacing, alignment (no animations yet)
2. **Responsive Second:** Ensure mobile/tablet/desktop all work correctly
3. **Animation Third:** Add tasteful transitions to enhance UX
4. **Accessibility Last:** Audit and fix keyboard nav, ARIA, focus management

This order ensures the UI is functional and correct before adding polish.

## Troubleshooting Common Issues

### Menu Alignment Issues
- **Problem:** Menu items not vertically centered
- **Fix:** Use `flexbox` with `align-items: center` or `grid` with `align-content: center`

### Submenu Not Animating
- **Problem:** Submenu appears/disappears instantly
- **Fix:** Animate `max-height` (0 → auto trick) or use `scaleY` transform

### Overflow on Mobile
- **Problem:** Content exceeds viewport width
- **Fix:** Add `overflow-x-hidden` to body/container, ensure `max-width: 100%` on wide elements

### Focus Ring Missing
- **Problem:** No visible focus indicator
- **Fix:** Add `focus:outline-2 focus:outline-brand-color` or custom `focus-visible` styles

### Animation Too Slow/Fast
- **Problem:** Transition feels laggy or abrupt
- **Fix:** Adjust duration (150–250ms sweet spot), check easing function

## Integration with Existing Design System

If the project has a design system or theme file:

1. **Read tokens first:** Look for `theme.ts`, `tokens.json`, `variables.scss`
2. **Use existing values:** Colors, spacing, typography, shadows, radii
3. **Extend carefully:** Only add new tokens if truly needed
4. **Propose tokens:** If spacing is inconsistent, suggest creating a scale

If no design system exists, propose creating one:
- `colors.ts`: Brand colors, grays, semantic colors (success, error, warning)
- `spacing.ts`: Consistent scale (4, 8, 12, 16, 24, 32, 48, 64...)
- `typography.ts`: Font families, sizes, weights, line-heights

## Advanced: Micro-Interactions

Once core polish is complete, consider adding subtle micro-interactions:

- **Button hover:** Slight scale (1.02) or color shift
- **Link hover:** Underline animation (left-to-right)
- **Card hover:** Lift effect (shadow + translateY)
- **Input focus:** Border color transition
- **Icon transitions:** Rotate chevron when menu expands

**Rule:** Keep it subtle. Good micro-interactions are barely noticeable but make the UI feel "premium."

## Resources

For deeper guidance, refer to:
- `CHECKLIST.md` – Complete UI/UX audit checklist
- `RECIPES.md` – Copy-paste friendly code patterns
- `TESTING.md` – Verification steps and manual tests
