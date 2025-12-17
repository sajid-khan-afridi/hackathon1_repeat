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

**ACTION:** Read the package.json file using the Read tool.

```
Read package.json
```

**Extract this information:**
- Framework: React, Next.js, Docusaurus, Vue, etc.
- Styling: Tailwind, CSS Modules, styled-components, SCSS
- Animation library: Framer Motion, React Spring, or CSS-only
- Build tool: Vite, Webpack, Parcel, etc.

### Step 1.2: Locate UI Files

**ACTION:** Use Glob tool to find UI-related files. Run these searches in parallel:

```
Glob: **/components/**/*.{tsx,jsx}
Glob: **/src/css/**/*.css
Glob: **/*.module.css
Glob: **/styles/**/*.{css,scss}
Glob: **/custom.css
Glob: **/theme/**/*
```

**Prioritize reading these files:**
1. Main CSS file (often `custom.css`, `globals.css`, `index.css`)
2. Component-specific styles
3. Layout components (Navbar, Header, Footer, Sidebar)
4. Theme configuration files

### Step 1.3: Analyze Key UI Files

**ACTION:** Read the 3-5 most important files you found. Look for:

**Alignment issues:**
- Inconsistent flexbox/grid usage (missing `align-items: center`)
- Icons and text not vertically aligned
- Menu items at different heights

**Spacing problems:**
- Arbitrary values like `13px`, `27px`, `19px` instead of system values (4, 8, 16, 24, 32)
- Inconsistent padding across similar components
- No spacing scale defined

**Animation gaps:**
- Missing `transition` properties on interactive elements
- Abrupt state changes (menus snapping open/closed)
- No hover states on buttons/links

**Responsive issues:**
- Fixed widths that don't scale (e.g., `width: 500px` instead of `max-width: 500px`)
- Missing breakpoints for mobile/tablet
- Horizontal overflow issues

**Accessibility gaps:**
- Missing focus states (`:focus`, `:focus-visible`)
- Low contrast colors
- No `@media (prefers-reduced-motion)` support
- Missing ARIA attributes on interactive components

## PHASE 2: Present Findings (ASK USER)

After analyzing the UI files, you MUST present your findings to the user before making any changes.

### Step 2.1: Summarize Your Findings

**ACTION:** Write a clear summary message to the user listing the 4 most impactful improvements you identified.

**Format your message like this:**

```
## UI/UX Analysis Complete

I analyzed your project and found several improvement opportunities. Here are the top 4:

**1. [Issue Category] - [Specific Problem]** (Priority: High/Medium/Low)
- **Problem:** [What's wrong, be specific with file:line references]
- **Impact:** [Why it matters - UX, accessibility, or visual quality]
- **Proposed Fix:** [Specific solution - what CSS/code changes you'll make]

**2. [Issue Category] - [Specific Problem]** (Priority: High/Medium/Low)
- **Problem:** [Description]
- **Impact:** [Impact]
- **Proposed Fix:** [Solution]

**3. [Issue Category] - [Specific Problem]** (Priority: High/Medium/Low)
- **Problem:** [Description]
- **Impact:** [Impact]
- **Proposed Fix:** [Solution]

**4. [Issue Category] - [Specific Problem]** (Priority: High/Medium/Low)
- **Problem:** [Description]
- **Impact:** [Impact]
- **Proposed Fix:** [Solution]
```

### Step 2.2: Ask User for Approval

**ACTION:** Immediately after presenting findings, use the AskUserQuestion tool like this:

```
AskUserQuestion with these parameters:
- questions: [
    {
      "question": "Which UI/UX improvements would you like me to apply?",
      "header": "Apply Fixes",
      "multiSelect": false,
      "options": [
        {
          "label": "Apply all 4 improvements (Recommended)",
          "description": "I'll implement all identified fixes in order of priority"
        },
        {
          "label": "Choose individually",
          "description": "I'll ask about each improvement one by one"
        },
        {
          "label": "Customize",
          "description": "Modify the proposed solutions before applying"
        },
        {
          "label": "Explain more",
          "description": "Get detailed explanations before deciding"
        }
      ]
    }
  ]
```

**IMPORTANT:** Do not proceed to Phase 3 until the user responds to this question.

## PHASE 3: User Selection Loop

**Only execute this phase if the user selected "Choose individually" in Phase 2.**

### Step 3.1: Present Each Improvement Individually

For each of the 4 improvements, use AskUserQuestion:

```
AskUserQuestion with parameters:
- questions: [
    {
      "question": "[Improvement #N]: [Brief description]. Should I apply this fix?",
      "header": "Fix N of 4",
      "multiSelect": false,
      "options": [
        {
          "label": "Yes, apply it",
          "description": "Implement this fix as proposed"
        },
        {
          "label": "Customize first",
          "description": "I want to modify the approach"
        },
        {
          "label": "Skip this one",
          "description": "Don't apply this fix"
        }
      ]
    }
  ]
```

### Step 3.2: Handle User Response

- If "Yes, apply it": Add to approved list, move to next improvement
- If "Customize first": Ask user what they want changed, then ask for approval again
- If "Skip this one": Mark as skipped, move to next improvement

### Step 3.3: After All 4 Improvements Reviewed

Summarize which improvements were approved:
```
I'll now apply these [N] improvements:
1. [Improvement name]
2. [Improvement name]
...

Skipped:
- [Improvement name] (if any)
```

Then proceed to Phase 4.

## PHASE 4: Implementation (Approved Changes Only)

**CRITICAL:** Only implement changes that the user approved in Phase 2 or Phase 3.

### Implementation Rules

1. **Use Edit tool, not Write**: Edit specific lines to create minimal diffs
2. **One file at a time**: Complete each file before moving to the next
3. **Follow existing patterns**: Match the project's code style and naming conventions
4. **Test incrementally**: If possible, verify each change doesn't break the build
5. **Document changes**: Note what you changed and where (file:line references)

### Step 4.1: Plan Implementation Order

Implement changes in this priority order:
1. **High priority fixes** first (accessibility, broken layouts)
2. **Medium priority** second (spacing, alignment)
3. **Low priority** last (animations, micro-interactions)

### Step 4.2: Implement Each Change

For each approved improvement:

**ACTION:** Use the Edit tool to modify the specific lines identified during analysis.

Example:
```
Edit file: src/css/custom.css
- Old: .navbar__item { padding: 13px; }
- New: .navbar__item { padding: 12px 16px; display: flex; align-items: center; }
```

**After each edit:**
- Verify the change matches your proposed fix
- Check that you didn't accidentally break syntax
- Move to the next change

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

After implementing ALL approved changes, you MUST verify that nothing broke.

### Step 5.1: Build Check

**ACTION:** Run the project's build command using the Bash tool.

**Determine the correct command:**
- Check package.json for the build script
- Common commands: `npm run build`, `yarn build`, `pnpm build`

**Execute:**
```
Bash: npm run build
(or whatever command is in package.json)
```

**Expected result:**
- ✅ Build completes successfully (exit code 0)
- ❌ Build fails: You MUST fix the errors before completing

If build fails:
1. Read the error message carefully
2. Identify which change caused the issue
3. Fix the syntax error or revert the problematic change
4. Run build again

### Step 5.2: Optional - Start Dev Server

If user wants to preview changes:
```
Bash: npm run dev
(or npm start, depending on project)
```

Tell user: "The dev server is running. You can preview the changes at http://localhost:[port]"

### Step 5.3: Report Changes Summary

**ACTION:** Write a comprehensive summary message to the user.

**Format:**
```markdown
## UI/UX Polish Complete ✅

### Applied Improvements

**1. [Improvement Category]**
- **Files changed:** `path/to/file.css:20-25`, `path/to/component.tsx:45`
- **What changed:** [Brief description of the changes]
- **Impact:** [What the user will notice]

**2. [Improvement Category]**
- **Files changed:** `path/to/file.css:100-110`
- **What changed:** [Brief description]
- **Impact:** [User impact]

**3. [Improvement Category]**
- **Files changed:** [...]
- **What changed:** [...]
- **Impact:** [...]

**4. [Improvement Category]**
- **Files changed:** [...]
- **What changed:** [...]
- **Impact:** [...]

### Verification Results

- ✅ **Build Status:** PASSED
- ✅ **Syntax:** No errors introduced
- ℹ️ **Files Modified:** [N] files
- ℹ️ **Lines Changed:** ~[N] lines

### Skipped Items

[List any improvements that were skipped and why, if applicable]

### Next Steps

1. Run `npm run dev` to preview the changes locally
2. Test responsive layouts on mobile/tablet/desktop
3. Verify keyboard navigation still works
4. Check that animations feel smooth (not jarring)

The UI/UX improvements are complete and ready for testing!
```

### Step 5.4: Mark Todo as Complete

Update your todo list to mark all implementation tasks as completed.

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
