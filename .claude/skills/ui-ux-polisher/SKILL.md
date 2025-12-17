---
name: ui-ux-polisher
description: >
  Systematically audit, analyze, and enhance UI/UX quality with visual analysis, interactive suggestions, and user-approved improvements. Use when user mentions: UI polish, menu alignment, responsiveness, animations, professional look, accessibility fixes, design improvements.
allowed-tools:
  - Read
  - Grep
  - Glob
  - Edit
  - Write
  - Bash
  - AskUserQuestion
  - WebFetch
---

# UI/UX Polisher Skill

You are now a UI/UX expert. Execute this skill by following the phases below systematically.

## PHASE 1: Discovery & Analysis (REQUIRED FIRST STEP)

### Step 1.1: Identify the Project Type

```bash
# Check for package.json to understand the project
```

Read `package.json` to determine:
- Framework: React, Next.js, Docusaurus, Vue, etc.
- Styling: Tailwind, CSS Modules, styled-components, SCSS
- Animation library: Framer Motion, React Spring, or CSS-only

### Step 1.2: Locate UI Files

Use Glob to find UI-related files:
- `**/components/**/*.{tsx,jsx,css,scss}`
- `**/src/css/**/*.css`
- `**/styles/**/*.{css,scss}`
- `**/src/theme/**/*`

### Step 1.3: Identify Current Issues

Read the main CSS files and key components. Look for:
1. **Alignment issues**: Inconsistent flexbox/grid usage
2. **Spacing problems**: Arbitrary values (13px, 27px) vs system (4, 8, 16, 24, 32)
3. **Animation gaps**: Missing transitions, abrupt state changes
4. **Responsive issues**: Fixed widths, missing breakpoints
5. **Accessibility gaps**: Missing focus states, contrast issues

## PHASE 2: Present Findings (ASK USER)

After analysis, use AskUserQuestion to present exactly 4 improvement suggestions:

```
Based on my analysis, I found these improvement opportunities:

**1. [Issue Name]** (Priority: High/Medium/Low)
- Problem: [Specific issue found]
- Impact: [Why it matters]
- Fix: [Proposed solution]

**2. [Issue Name]** ...
**3. [Issue Name]** ...
**4. [Issue Name]** ...

Which improvements would you like me to apply?
A) All of them
B) Let me choose individually
C) Customize specific ones
D) Skip and explain more
```

Use this AskUserQuestion format:
- Question: "Which UI/UX improvements would you like to apply?"
- Header: "Improvements"
- Options:
  - "Apply all 4 improvements" - Apply all identified fixes
  - "Choose individually" - I'll ask about each one separately
  - "Customize" - Modify the proposed solutions
  - "Explain more" - Get more details before deciding

## PHASE 3: User Selection Loop

If user chose "Choose individually", loop through each improvement:

For each improvement, ask:
- Question: "[Improvement name]: [Brief description]. Apply this fix?"
- Header: "Fix #N"
- Options:
  - "Yes" - Apply as proposed
  - "Customize" - Modify the approach
  - "Skip" - Don't apply this fix

## PHASE 4: Implementation (Approved Changes Only)

### Implementation Rules

1. **Minimal diffs only**: Edit specific lines, don't rewrite files
2. **Follow existing patterns**: Use the project's naming conventions
3. **Preserve functionality**: Never break existing routes or features
4. **One cluster at a time**: Group related changes, verify after each

### CSS Changes Standard

When editing CSS:
```css
/* Animation tokens - standardize on these */
--animation-fast: 150ms;
--animation-base: 250ms;
--animation-slow: 350ms;
--animation-easing: cubic-bezier(0.4, 0, 0.2, 1);

/* Always include reduced motion */
@media (prefers-reduced-motion: reduce) {
  * {
    animation-duration: 0.01ms !important;
    transition-duration: 0.01ms !important;
  }
}
```

### Component Changes Standard

When editing components:
- Add `aria-*` attributes for accessibility
- Use semantic HTML (`<button>`, `<nav>`, not `<div onClick>`)
- Ensure focus indicators are visible
- Touch targets minimum 44x44px

## PHASE 5: Verification (REQUIRED)

After implementing changes:

### Step 5.1: Build Check
```bash
npm run build
```

If build fails, fix issues before proceeding.

### Step 5.2: Report Changes

Output this summary:
```markdown
### UI/UX Polish Complete

**Applied Improvements:**
1. [Improvement 1] - Files: `path/file.css` (lines X-Y)
2. [Improvement 2] - Files: `path/component.tsx` (lines X-Y)
...

**Changes Made:**
- [Specific change 1]
- [Specific change 2]
...

**Verification:**
- Build: [PASS/FAIL]
- [Any other checks performed]

**Skipped:** (if any)
- [Reason for skipping]
```

## QUICK REFERENCE: Common Fixes

### Fix: Navbar Alignment
```css
.navbar__items--right {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.navbar__items--right > * {
  height: 40px;
  display: flex;
  align-items: center;
}
```

### Fix: Button Hover Animation
```css
.button {
  transition: transform var(--animation-fast) var(--animation-easing),
              box-shadow var(--animation-fast) var(--animation-easing);
}

.button:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

.button:active {
  transform: translateY(0);
}
```

### Fix: Focus Visibility
```css
*:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}
```

### Fix: Responsive Mobile Menu
```css
@media (max-width: 996px) {
  .navbar__items {
    display: none;
  }

  .navbar__toggle {
    display: flex;
  }
}
```

### Fix: Smooth Submenu Expand
```css
.menu__list {
  max-height: 0;
  overflow: hidden;
  opacity: 0;
  transition: max-height var(--animation-base) var(--animation-easing),
              opacity var(--animation-base) var(--animation-easing);
}

.menu__list--expanded {
  max-height: 500px;
  opacity: 1;
}
```

## CONSTRAINTS

- Never change brand colors without explicit user approval
- Never remove existing accessibility features
- Never add heavy animation libraries if project uses CSS-only
- Never use fixed pixel widths that break responsiveness
- Never skip the user approval phase
- Maximum 4 improvements per session to avoid overwhelm
- Always verify build passes after changes
