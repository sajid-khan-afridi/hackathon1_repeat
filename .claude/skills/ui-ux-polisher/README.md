# UI/UX Polisher Skill (Enhanced with Playwright MCP)

**Purpose:** Systematically audit, diagnose, and enhance UI/UX quality with professional polish, consistent design patterns, smooth animations, and accessibility compliance using automated Playwright analysis.

## 🆕 Enhanced Workflow with Playwright MCP

This skill now includes **intelligent website analysis** using Playwright MCP server before making any changes.

### Phase 1: Automated Analysis (NEW)

1. **Launch Local Development Server**
   ```bash
   npm run start -- --port 3000
   ```

2. **Playwright MCP Analysis**
   - Navigate to homepage, key pages, and interactive components
   - Capture screenshots of current state
   - Analyze:     - Layout alignment issues (navbar, buttons, spacing)
     - Visual hierarchy problems
     - Animation quality and consistency
     - Responsive design at multiple breakpoints (mobile 375px, tablet 768px, desktop 1440px+)
     - Accessibility issues (contrast, focus states, ARIA)
     - Brand consistency (logos, icons, colors)

3. **Generate Analysis Report**
   Create a markdown report with:
   - Screenshots of identified issues
   - Specific CSS selectors affected
   - Severity ratings (Critical, High, Medium, Low)

### Phase 2: Interactive Improvement Suggestions

Based on Playwright analysis, Claude presents **4 suggested improvements** with:

**Example Questions:**
```markdown
## Suggested UI/UX Improvements

I've analyzed your website and identified these opportunities:

**1. Navbar Alignment Issues** (Priority: High)
- Issue: GitHub button and dark mode toggle are misaligned
- Impact: Unprofessional appearance, inconsistent spacing
- Fix: Align both buttons to 40px height with flexbox centering
- Do you want to fix this? [Yes / No / Customize]

**2. Brand Identity** (Priority: Medium)
- Issue: Generic Docusaurus icon doesn't match robotics textbook theme
- Impact: Missed branding opportunity
- Fix: Replace with custom robotics/AI icon (gear + robot motif)
- Do you want to fix this? [Yes / No / Customize]

**3. Chatbot Styling** (Priority: High)
- Issue: Chatbot widget lacks professional polish
- Impact: Doesn't match site's premium feel
- Fix: Add glassmorphism effect, smooth animations, better typography
- Do you want to fix this? [Yes / No / Customize]

**4. Homepage Animations** (Priority: Low)
- Issue: Animations could be more synchronized
- Impact: Slightly choppy user experience
- Fix: Standardize animation timing to 3-tier system (150ms/250ms/350ms)
- Do you want to fix this? [Yes / No / Customize]
```

### Phase 3: User Feedback Integration

User can respond with:
- **"Yes"** → Apply the suggested fix as-is
- **"No"** → Skip this improvement
- **"Customize"** → Ask follow-up questions for specific preferences

**Example Customization Flow:**
```markdown
User: "Customize the chatbot styling"

Claude:
> Great! Let's customize the chatbot. Which style do you prefer?
>
> A) **Glassmorphism** (frosted glass effect, modern)
> B) **Neumorphism** (soft shadows, tactile feel)
> C) **Minimalist** (flat design, clean lines)
> D) **Material Design** (cards, elevation, depth)
>
> Also, what accent color? [Keep current blue / Purple / Green / Custom]
```

### Phase 4: Targeted Implementation

Only implement the improvements user confirmed:
- Make minimal, surgical changes
- Preserve all existing functionality
- Follow existing code patterns
- Test after each change

### Phase 5: Post-Implementation Verification

1. **Automated Testing**
   - Build: `npm run build`
   - Lighthouse audit scores
   - Accessibility scan

2. **Visual Regression**
   - Playwright screenshots (before/after)
   - Side-by-side comparison

3. **User Confirmation**
   - Show screenshots of improvements
   - Ask: "Does this match your expectations?"

---

## When to Use This Skill

Invoke when user mentions:
- "UI/UX polish", "professional look", "improve design"
- "menu alignment", "navbar issues", "button alignment"
- "responsiveness", "mobile layout", "breakpoint issues"
- "animations", "smooth transitions", "micro-interactions"
- "chatbot design", "widget styling", "component polish"
- "analyze my website", "what can be improved"

---

## Integration with Playwright MCP

### Prerequisites
```json
// .mcp.json (if using Playwright MCP server)
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-playwright"]
    }
  }
}
```

### Playwright Analysis Commands

```javascript
// 1. Navigate and capture
await page.goto('http://localhost:3000');
await page.screenshot({ path: 'analysis/homepage.png' });

// 2. Test responsiveness
await page.setViewportSize({ width: 375, height: 667 }); // Mobile
await page.screenshot({ path: 'analysis/mobile.png' });

await page.setViewportSize({ width: 768, height: 1024 }); // Tablet
await page.screenshot({ path: 'analysis/tablet.png' });

// 3. Check navbar alignment
const navbar = await page.locator('.navbar__items--right');
const boundingBox = await navbar.boundingBox();
console.log('Navbar alignment:', boundingBox);

// 4. Test dark mode toggle
await page.click('[aria-label*="dark mode"]');
await page.screenshot({ path: 'analysis/dark-mode.png' });

// 5. Test chatbot widget
await page.click('.floatingButton'); // Floating chat button
await page.waitForSelector('.chatbotWidget', { state: 'visible' });
await page.screenshot({ path: 'analysis/chatbot-open.png' });
```

---

## Core UI/UX Principles (Unchanged)

### Animation Standards
- **Fast**: 150ms (micro-interactions)
- **Base**: 250ms (standard transitions)
- **Slow**: 350ms (complex animations)
- **Easing**: `cubic-bezier(0.4, 0, 0.2, 1)` (Material Design)
- **Reduced Motion**: Always provide instant fallbacks

### Typography
- **Headings**: line-height 1.2-1.3, letter-spacing -0.015em to -0.025em
- **Body**: line-height 1.6-1.7
- **Hierarchy**: Use size + weight + spacing for visual layers

### Accessibility (WCAG 2.1 AA)
- **Contrast**: ≥4.5:1 for text, ≥3:1 for large text
- **Touch Targets**: Minimum 44×44px
- **Keyboard Nav**: Tab order, focus indicators, Escape to close
- **ARIA**: Proper labels, roles, states

### Responsive Design
- **Mobile**: < 640px (sm)
- **Tablet**: 640px – 1024px (md/lg)
- **Desktop**: ≥ 1024px (xl)

---

## Output Format

After implementing user-approved changes:

```markdown
### ✅ [Improvement Name] Complete

**Files Changed:**
- `src/css/custom.css` (lines 857-913: navbar alignment fixes)
- `src/components/FloatingChatButton/styles.module.css` (lines 24-26: animation tokens)

**Changes Made:**
1. Fixed GitHub button and dark mode toggle alignment (40px height, flexbox centering)
2. Added standardized animation transitions (150ms/250ms/350ms)
3. Improved hover states with scale transform (1.1x)

**Screenshots:**
- Before: `analysis/navbar-before.png`
- After: `analysis/navbar-after.png`

**Verification:**
- ✅ Build successful
- ✅ Lighthouse Performance: 95/100
- ✅ Lighthouse Accessibility: 100/100
- ✅ Tested on mobile (375px), tablet (768px), desktop (1440px)
- ✅ Keyboard navigation works
- ✅ Reduced motion respected
```

---

## Example Full Workflow

```markdown
User: "Use UI/UX polish skill"

Claude:
1. Starting local dev server...
2. Analyzing website with Playwright MCP...
   ✅ Homepage analyzed
   ✅ Navbar analyzed
   ✅ Chatbot widget analyzed
   ✅ Responsive breakpoints tested
   ✅ Accessibility scan complete

3. **Analysis Results:**
   Found 4 improvement opportunities:
   - Navbar alignment (High priority)
   - Brand icon (Medium priority)
   - Chatbot styling (High priority)
   - Animation timing (Low priority)

   [Shows 4 suggested improvements with descriptions]

   Which improvements would you like to apply?
   A) All of them
   B) Let me choose individually
   C) Customize each one

User: "B - let me choose"

Claude:
> **Improvement 1: Navbar Alignment**
> Fix GitHub button and dark mode toggle alignment?
> [Yes / No / Customize]

User: "Yes"

Claude:
✅ Navbar alignment fixed!
[Shows before/after screenshots]

> **Improvement 2: Brand Icon**
> Replace Docusaurus icon with robotics theme icon?
> [Yes / No / Customize]

User: "Customize"

Claude:
> Which style icon do you prefer?
> A) Gear + Robot (industrial)
> B) Circuit + AI (tech-forward)
> C) Book + Robot (education-focused)

User: "C"

Claude:
✅ Created education-focused robotics icon!
[Shows new icon]

[Continues with remaining improvements...]

Final Report:
✅ Applied 3/4 improvements
✅ All builds successful
✅ Screenshots saved to analysis/
✅ Changes ready to commit

Would you like to deploy these changes?
```

---

## Advanced Features

### Brand Consistency Check
- Verify color palette consistency (primary, secondary, accents)
- Check icon style uniformity
- Validate font family usage
- Ensure logo/brand mark quality

### Performance Optimization
- Lazy load images
- Optimize animation performance (`will-change`, GPU acceleration)
- Minimize layout shifts (CLS)
- Reduce animation jank (60fps target)

### Dark Mode Validation
- Test all color combinations in dark theme
- Ensure sufficient contrast in both modes
- Verify animations work in dark mode
- Check icon visibility

---

## Troubleshooting

**Playwright MCP not available:**
- Fall back to manual audit (original workflow)
- Use visual inspection + code review
- Rely on user feedback for issue identification

**Dev server won't start:**
- Check port 3000 availability
- Try alternative port: `npm run start -- --port 3001`
- Use production build analysis if needed

**Screenshots not saving:**
- Verify `analysis/` directory exists
- Check file system permissions
- Use browser DevTools as backup

---

## Integration Points

- **Before**: Manual UI review, ad-hoc fixes
- **After**: Automated analysis → Guided improvements → User approval → Targeted fixes
- **Benefits**: Faster, more accurate, user-driven, documented changes
