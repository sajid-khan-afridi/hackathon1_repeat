# UI/UX Polish Checklist

Use this checklist to systematically audit and enhance UI quality. Check items as you complete them.

## 🎯 Layout & Structure

### Alignment
- [ ] Menu items vertically centered with consistent spacing
- [ ] Icons aligned with text (vertical center, consistent left/right padding)
- [ ] Button text centered within button bounds
- [ ] Form labels aligned with inputs
- [ ] Grid/flex layouts use consistent gaps

### Spacing System
- [ ] Consistent spacing scale used (e.g., 4px, 8px, 16px, 24px, 32px)
- [ ] Padding consistent across similar components (all buttons, all cards)
- [ ] Margin between sections follows predictable rhythm
- [ ] No arbitrary spacing values (avoid `padding: 13px` without reason)
- [ ] Whitespace used intentionally to create visual hierarchy

### Container Widths
- [ ] Main content has max-width (avoid text lines > 80ch)
- [ ] Containers centered or aligned consistently
- [ ] Full-width sections span correctly without overflow
- [ ] Cards/panels use consistent width/max-width

## 📱 Responsive Design

### Breakpoints
- [ ] Mobile (< 640px): Single column, hamburger menu, touch-friendly
- [ ] Tablet (640px – 1024px): Adaptive layout, may show sidebar
- [ ] Desktop (≥ 1024px): Full layout with all navigation visible

### Mobile-Specific
- [ ] No horizontal overflow (`overflow-x: hidden` on body/container)
- [ ] Touch targets ≥ 44×44px (iOS), prefer 48×48px
- [ ] Hamburger menu accessible and functional
- [ ] Font sizes readable (body ≥ 16px, avoid tiny text)
- [ ] Images/videos scale correctly (`max-width: 100%`)

### Tablet-Specific
- [ ] Layout adapts gracefully (not just scaled-up mobile)
- [ ] Navigation clear (may be collapsed or visible)
- [ ] Touch + mouse interaction both work

### Desktop-Specific
- [ ] Full navigation visible (navbar, sidebar, breadcrumbs)
- [ ] Multi-column layouts utilized where appropriate
- [ ] Hover states visible and functional

## 🎨 Typography

### Font Scale
- [ ] Consistent heading hierarchy (H1 > H2 > H3 > body > small)
- [ ] No more than 3-4 font sizes per view
- [ ] Hero/display text appropriately large (2.5rem – 4rem)
- [ ] Body text readable (1rem / 16px minimum)

### Line Height
- [ ] Headings: 1.2 – 1.3
- [ ] Body text: 1.5 – 1.7
- [ ] No line-height < 1.2 (too tight) or > 2 (too loose)

### Font Weights
- [ ] Consistent weight usage (e.g., 400 regular, 600 semibold, 700 bold)
- [ ] Avoid excessive weights (≤ 3 weights per design)
- [ ] Hierarchy uses size + weight + spacing

### Contrast
- [ ] Text on background ≥ 4.5:1 contrast (WCAG AA)
- [ ] Large text (≥ 18pt) ≥ 3:1 contrast
- [ ] Verify with WebAIM Contrast Checker or browser DevTools

## 🎭 Visual Hierarchy

### Priority Levels
- [ ] Primary actions stand out (larger, bolder, colored)
- [ ] Secondary actions less prominent (smaller, outlined, muted)
- [ ] Tertiary actions subtle (text links, icon buttons)
- [ ] Clear visual flow (eye naturally follows most important → least important)

### Color Usage
- [ ] Consistent color palette (primary, secondary, accent, neutrals)
- [ ] Semantic colors for states (success green, error red, warning yellow)
- [ ] Color not the only indicator (use icons/text for accessibility)
- [ ] Brand colors used consistently

### Grouping & Separation
- [ ] Related items grouped visually (cards, sections, panels)
- [ ] Clear separation between distinct sections (borders, spacing, background color)
- [ ] Hierarchy created with size, weight, color, and spacing

## ✨ Animations & Transitions

### Menu Animations
- [ ] Menu open/close: smooth 150–250ms transition
- [ ] Submenu expand/collapse: animated (height, opacity, or transform)
- [ ] Animation easing feels natural (`ease-out`, `cubic-bezier(0.4, 0, 0.2, 1)`)
- [ ] No jarring snaps or abrupt changes

### Micro-Interactions
- [ ] Button hover: subtle scale or color shift
- [ ] Link hover: underline animation or color change
- [ ] Card hover: lift effect (shadow, translateY)
- [ ] Input focus: border color transition

### Performance
- [ ] Animations use `transform` and `opacity` (GPU-accelerated)
- [ ] Avoid animating `width`, `height`, `top`, `left` directly (causes reflow)
- [ ] Use `will-change` sparingly (only for elements actively animating)

### Reduced Motion
- [ ] `@media (prefers-reduced-motion: reduce)` implemented
- [ ] Reduced motion: instant transitions (duration: 0ms) or no animation
- [ ] Essential animations still functional (don't break UX if disabled)

## ♿ Accessibility

### Keyboard Navigation
- [ ] All interactive elements focusable (buttons, links, inputs, custom controls)
- [ ] Tab order follows visual layout (logical flow)
- [ ] Focus indicator visible (outline, ring, or custom style)
- [ ] Escape key closes modals/menus/dialogs
- [ ] Enter/Space activates buttons/links
- [ ] Arrow keys navigate menus (if applicable)

### ARIA & Semantic HTML
- [ ] Use semantic tags (`<nav>`, `<button>`, `<main>`, `<aside>`, `<header>`, `<footer>`)
- [ ] Avoid `<div onClick>` (use `<button>` instead)
- [ ] Menus use `aria-expanded`, `aria-controls`, `aria-haspopup`
- [ ] Dropdowns have `role="menu"` and `role="menuitem"` (or `role="listbox"` for selects)
- [ ] Icon-only buttons have `aria-label`
- [ ] Images have `alt` text (or `alt=""` if decorative)

### Focus Management
- [ ] Opening modal: focus moves to modal (first interactive element or close button)
- [ ] Closing modal: focus returns to trigger element
- [ ] Focus trapped inside modals (Tab doesn't escape to page behind)
- [ ] Skip links provided for keyboard users ("Skip to main content")

### Screen Reader Support
- [ ] Content announced in logical order
- [ ] Dynamic content changes announced (`aria-live` regions if needed)
- [ ] Form errors associated with inputs (`aria-describedby`)
- [ ] Status messages communicated (loading, success, error)

## 🧪 Testing & Verification

### Build & Lint
- [ ] `npm run lint` (or equivalent) passes with no errors
- [ ] `npm run type-check` (TypeScript) passes
- [ ] `npm run build` succeeds without errors
- [ ] No console errors in browser DevTools

### Visual Testing (Manual)
- [ ] Test on mobile viewport (375px width minimum)
- [ ] Test on tablet viewport (768px width)
- [ ] Test on desktop viewport (1440px+ width)
- [ ] Test on different browsers (Chrome, Firefox, Safari if possible)
- [ ] Dark mode (if applicable) renders correctly

### Interaction Testing
- [ ] Click all buttons/links (ensure they work)
- [ ] Open/close all menus and submenus
- [ ] Test hamburger menu on mobile
- [ ] Fill out forms (if applicable) and submit
- [ ] Verify animations feel smooth (60fps, no jank)

### Keyboard-Only Testing
- [ ] Unplug mouse (or don't touch trackpad)
- [ ] Navigate entire page using only Tab, Enter, Escape, Arrow keys
- [ ] Verify focus ring always visible
- [ ] Ensure all functionality accessible

### Reduced Motion Testing
- [ ] Enable "Reduce motion" in OS settings:
  - macOS: System Preferences → Accessibility → Display → Reduce motion
  - Windows: Settings → Ease of Access → Display → Show animations
  - iOS/Android: Accessibility settings
- [ ] Verify animations become instant or minimal
- [ ] UI still functional with reduced motion

### Performance
- [ ] Lighthouse audit score ≥ 90 (Performance, Accessibility, Best Practices)
- [ ] No layout shifts (CLS < 0.1)
- [ ] Fast interaction (FID < 100ms)
- [ ] Smooth scrolling and animations (no dropped frames)

## 🎁 Nice-to-Have Enhancements

### Polish Layer 1 (High Impact)
- [ ] Consistent button styles (primary, secondary, tertiary)
- [ ] Hover states on all interactive elements
- [ ] Loading states for async actions (spinners, skeletons)
- [ ] Error states clearly communicated (validation, network errors)
- [ ] Success feedback (toasts, inline messages)

### Polish Layer 2 (Medium Impact)
- [ ] Smooth page transitions (if using client-side routing)
- [ ] Skeleton screens for loading content
- [ ] Empty states with helpful messaging
- [ ] Tooltips for icon-only buttons
- [ ] Breadcrumbs for navigation context

### Polish Layer 3 (Low Impact, High Delight)
- [ ] Subtle shadows for depth (cards, modals, dropdowns)
- [ ] Border radius consistency (all buttons, cards, inputs use same radii)
- [ ] Micro-interactions (button press, link hover, icon animation)
- [ ] Smooth scroll behavior (`scroll-behavior: smooth` or JS equivalent)
- [ ] Custom scrollbars (if brand requires, optional)

## 📋 Cluster Completion Tracker

Use this to track implementation progress:

- [ ] **Cluster 1: Layout & Spacing**
  - Alignment fixes
  - Spacing system applied
  - Container widths adjusted

- [ ] **Cluster 2: Responsive Design**
  - Mobile layout functional
  - Tablet layout functional
  - Desktop layout optimized
  - No horizontal overflow

- [ ] **Cluster 3: Typography**
  - Font scale consistent
  - Line heights corrected
  - Contrast verified
  - Visual hierarchy clear

- [ ] **Cluster 4: Animations**
  - Menu animations added
  - Submenu animations added
  - Micro-interactions polished
  - Reduced motion implemented

- [ ] **Cluster 5: Accessibility**
  - Keyboard navigation works
  - ARIA attributes correct
  - Focus management implemented
  - Screen reader tested

- [ ] **Cluster 6: Testing & Verification**
  - Build passes
  - Visual testing complete
  - Interaction testing complete
  - Keyboard testing complete
  - Reduced motion tested

---

**Usage:** Copy this checklist into your task tracker or use it as a guide during UI/UX polish sessions. Check items systematically to ensure comprehensive coverage.
