# UI/UX Polisher Skill

A prompt-based skill that systematically audits, analyzes, and enhances UI/UX quality through interactive user feedback.

## Overview

This skill guides Claude through a structured workflow to:
1. **Discover** - Identify project framework, styling approach, and UI files
2. **Analyze** - Find alignment, spacing, animation, and accessibility issues
3. **Present** - Show up to 4 improvements with priorities for user approval
4. **Implement** - Apply only user-approved changes with minimal diffs
5. **Verify** - Run build and report changes

## Usage

Invoke this skill when users mention:
- "UI/UX polish" / "polish the UI"
- "improve design" / "professional look"
- "menu alignment" / "navbar issues"
- "responsiveness" / "mobile layout"
- "add animations" / "smooth transitions"
- "accessibility improvements"

### Example Invocation

```
User: "Use the UI/UX polish skill on this project"
```

Claude will then:
1. Read `package.json` to identify the framework
2. Search for CSS and component files
3. Identify issues and present 4 suggestions
4. Ask user which improvements to apply
5. Implement approved changes
6. Verify build passes

## Workflow Phases

### Phase 1: Discovery
- Read `package.json` for framework detection
- Glob for `**/components/**/*`, `**/src/css/**/*`, `**/styles/**/*`
- Identify: React/Next/Docusaurus, Tailwind/CSS Modules, animation libraries

### Phase 2: Analysis
- Read main CSS files and key components
- Look for:
  - Alignment issues (flexbox/grid inconsistencies)
  - Spacing problems (arbitrary vs systematic values)
  - Animation gaps (missing transitions)
  - Responsive issues (fixed widths, missing breakpoints)
  - Accessibility gaps (focus states, contrast)

### Phase 3: Presentation
- Present exactly 4 improvement suggestions
- Each with: Problem, Impact, Proposed Fix, Priority
- Use `AskUserQuestion` for user approval:
  - "Apply all 4 improvements"
  - "Choose individually"
  - "Customize"
  - "Explain more"

### Phase 4: Implementation
- Apply only approved changes
- Use minimal diffs (Edit tool, not rewrite)
- Follow existing code patterns
- One cluster at a time

### Phase 5: Verification
- Run `npm run build`
- Report all changes with file paths and line numbers
- Confirm build status

## Standards

### Animation Tokens
```css
--animation-fast: 150ms;
--animation-base: 250ms;
--animation-slow: 350ms;
--animation-easing: cubic-bezier(0.4, 0, 0.2, 1);
```

### Spacing Scale
`4px, 8px, 12px, 16px, 24px, 32px, 48px, 64px`

### Accessibility
- Minimum touch target: 44x44px
- Minimum contrast ratio: 4.5:1
- Focus indicators required
- Keyboard navigation required

### Breakpoints
- Mobile: < 640px
- Tablet: 640px - 1024px
- Desktop: >= 1024px

## Constraints

- Never change brand colors without explicit user approval
- Never remove existing accessibility features
- Never add heavy animation libraries if project uses CSS-only
- Never use fixed pixel widths that break responsiveness
- Never skip the user approval phase
- Maximum 4 improvements per session
- Always verify build passes after changes

## Files

| File | Purpose |
|------|---------|
| `SKILL.md` | Execution instructions (the prompt) |
| `skill.json` | Skill metadata and configuration |
| `index.js` | Entry point and utility functions |
| `README.md` | This documentation |
| `CHECKLIST.md` | Detailed audit checklist reference |
| `RECIPES.md` | Copy-paste code patterns |
| `TESTING.md` | Verification guide |

## Reference Files

### CHECKLIST.md
Complete audit checklist for:
- Layout & Structure
- Responsive Design
- Typography
- Visual Hierarchy
- Animations & Transitions
- Accessibility
- Testing & Verification

### RECIPES.md
Copy-paste patterns for:
- Responsive navbar with mobile menu
- Side drawer with overlay
- Submenu expand/collapse
- Accessible disclosure menu
- Button hover effects
- Link underline animation
- Card hover effects
- Loading skeletons
- Focus ring consistency

### TESTING.md
Verification steps for:
- Build & lint checks
- Browser DevTools checks
- Responsive testing
- Keyboard navigation testing
- Screen reader testing
- Reduced motion testing
- Cross-browser testing

## Integration

This skill is part of the `ui-component-agent` agent and can be used alongside:
- `react-components` - Generate new React components
- `contextual-keyword-chips` - Add keyword extraction UI
- `docusaurus-init` - Initialize Docusaurus projects

## Changelog

### v3.0.0
- Complete rewrite for actionable execution
- Added structured 5-phase workflow
- Integrated AskUserQuestion for user approval
- Added build verification requirement
- Consolidated documentation

### v2.0.0
- Added Playwright MCP integration concept
- Enhanced with interactive suggestions

### v1.0.0
- Initial release with basic audit workflow
