# Research: Phase 5 Translation

**Feature**: 006-translation
**Date**: 2025-12-27
**Status**: Complete

## Research Tasks

### 1. Docusaurus i18n Integration

**Decision**: Use Docusaurus built-in i18n with static file generation

**Rationale**:
- Docusaurus v3 has native i18n support with `docs/i18n/{locale}/` directory structure
- Static generation aligns with build-time translation strategy (FR-015)
- Automatic locale detection and URL routing (`/ur/` prefix for Urdu)
- Theme components can be localized via JSON files

**Alternatives Considered**:
- Custom translation layer: Rejected - reinvents existing functionality
- Runtime translation loading: Rejected - adds latency, complexity, and runtime API dependency

**Implementation Pattern**:
```text
docs/i18n/ur/
├── docusaurus-plugin-content-docs/
│   └── current/
│       ├── module-1/
│       │   ├── chapter-1.mdx
│       │   └── chapter-2.mdx
│       └── module-2/
│           └── ...
└── docusaurus-theme-classic/
    └── navbar.json
```

### 2. RTL Layout Best Practices

**Decision**: CSS logical properties + `dir="rtl"` attribute + Docusaurus RTL theme

**Rationale**:
- CSS logical properties (`margin-inline-start`, `padding-inline-end`) automatically flip for RTL
- Docusaurus supports `direction: rtl` in theme config
- Avoids hardcoded `left`/`right` values that break in RTL
- BiDi text handling with `unicode-bidi: embed` for mixed content

**Alternatives Considered**:
- Manual CSS overrides for RTL: Rejected - error-prone, hard to maintain
- Separate RTL stylesheet: Rejected - duplication, sync issues

**Implementation Pattern**:
```css
/* Use logical properties */
.component {
  margin-inline-start: 1rem;  /* instead of margin-left */
  padding-inline-end: 0.5rem; /* instead of padding-right */
  text-align: start;          /* instead of left */
}

/* RTL-specific overrides only when necessary */
[dir="rtl"] .icon-arrow {
  transform: scaleX(-1); /* Flip directional icons */
}
```

### 3. urdu-translator Skill Integration

**Decision**: Invoke `urdu-translator` skill during CI/CD build with technical term protection

**Rationale**:
- Skill already exists in `.claude/skills/urdu-translator/`
- Build-time invocation keeps API keys secure (not in client bundle)
- Technical term protection via pre-processing: wrap terms in `<preserve>` markers

**Alternatives Considered**:
- Direct OpenAI API: Rejected - skill abstracts API details, handles rate limiting
- External translation service (Google Translate, DeepL): Rejected - less control over technical term preservation

**Implementation Pattern**:
```typescript
// Pre-process: protect technical terms
const protectedContent = content.replace(
  /(ROS|Python|NVIDIA Isaac|Gazebo|URDF|TensorFlow|PyTorch)/g,
  '<preserve>$1</preserve>'
);

// Invoke urdu-translator skill
const translated = await urduTranslator.translate(protectedContent);

// Post-process: restore technical terms
const final = translated.replace(/<preserve>(.*?)<\/preserve>/g, '$1');
```

### 4. Technical Term Glossary Storage

**Decision**: JSON file in repository + optional Neon Postgres sync for authenticated users

**Rationale**:
- Static JSON file (`src/data/glossary.json`) for build-time validation and client rendering
- Simple, no runtime database dependency for glossary display
- Neon Postgres stores user preferences only (language selection)

**Alternatives Considered**:
- Glossary in Neon Postgres only: Rejected - adds latency, requires API for glossary page
- Markdown glossary file: Rejected - harder to programmatically validate

**Implementation Pattern**:
```json
{
  "terms": [
    {
      "english": "ROS",
      "urduTransliteration": "آر او ایس",
      "definition": "Robot Operating System - a flexible framework for writing robot software",
      "definitionUrdu": "روبوٹ آپریٹنگ سسٹم - روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک",
      "category": "Robotics Frameworks",
      "relatedTerms": ["ROS 2", "roscore", "rosnode"]
    }
  ]
}
```

### 5. Language Toggle Implementation

**Decision**: React Context + localStorage + profile sync hook

**Rationale**:
- React Context provides global language state without prop drilling
- localStorage for guest users (FR-006)
- Profile sync for authenticated users (FR-007)
- Instant toggle response (< 200ms) via pre-loaded translations

**Alternatives Considered**:
- URL-based locale only: Rejected - requires page reload, poor UX
- Cookie-based persistence: Rejected - localStorage is simpler, no server-side rendering

**Implementation Pattern**:
```typescript
// LanguageContext.tsx
const LanguageContext = createContext<{
  locale: 'en' | 'ur';
  setLocale: (locale: 'en' | 'ur') => void;
}>({ locale: 'en', setLocale: () => {} });

// useLanguagePreference.ts
function useLanguagePreference() {
  const { user } = useAuth();
  const [locale, setLocaleState] = useState(() =>
    localStorage.getItem('locale') || 'en'
  );

  const setLocale = async (newLocale: 'en' | 'ur') => {
    localStorage.setItem('locale', newLocale);
    setLocaleState(newLocale);
    if (user) {
      await updateUserProfile({ preferredLanguage: newLocale });
    }
  };

  return { locale, setLocale };
}
```

### 6. Visual Regression Testing Strategy

**Decision**: Playwright with screenshot comparison on 3 viewports

**Rationale**:
- Playwright has built-in screenshot comparison (`toHaveScreenshot`)
- Test both LTR (English) and RTL (Urdu) on same pages
- 3 viewports as per SC-005: 320px (mobile), 768px (tablet), 1024px (desktop)

**Alternatives Considered**:
- Percy/Chromatic: Rejected - adds external service cost, overkill for 10 chapters
- Manual QA only: Rejected - not automated, doesn't catch regressions

**Implementation Pattern**:
```typescript
// rtl-regression.spec.ts
const viewports = [
  { width: 320, height: 568, name: 'mobile' },
  { width: 768, height: 1024, name: 'tablet' },
  { width: 1024, height: 768, name: 'desktop' },
];

for (const viewport of viewports) {
  test(`RTL layout - ${viewport.name}`, async ({ page }) => {
    await page.setViewportSize(viewport);
    await page.goto('/ur/module-1/chapter-1');
    await expect(page).toHaveScreenshot(`chapter-1-${viewport.name}-rtl.png`);
  });
}
```

### 7. Content Hash for Cache Invalidation

**Decision**: MD5 hash of source MDX content stored in translation metadata

**Rationale**:
- Simple, deterministic hash identifies content changes
- Stored alongside translation: if hash differs, regenerate
- Build script compares hashes before translation to skip unchanged chapters

**Alternatives Considered**:
- Git commit hash: Rejected - changes even for unrelated edits
- File modification timestamp: Rejected - not deterministic across environments

**Implementation Pattern**:
```typescript
// generate-content-hash.ts
import { createHash } from 'crypto';
import { readFileSync } from 'fs';

function getContentHash(filePath: string): string {
  const content = readFileSync(filePath, 'utf-8');
  return createHash('md5').update(content).digest('hex');
}

// Translation metadata
interface TranslationMeta {
  sourceHash: string;
  translatedAt: string;
  chapterId: string;
}
```

### 8. Accessibility Implementation

**Decision**: WCAG 2.1 AA compliance with ARIA live regions for language changes

**Rationale**:
- `aria-live="polite"` announces language change to screen readers
- `<html lang="ur" dir="rtl">` for proper RTL screen reader behavior
- Focus management after language toggle (maintain focus position)

**Alternatives Considered**:
- WCAG 2.1 AAA: Rejected - exceeds project scope, AA is constitution requirement
- No ARIA live regions: Rejected - screen reader users wouldn't know language changed

**Implementation Pattern**:
```tsx
// LanguageToggle/index.tsx
<button
  onClick={() => setLocale(locale === 'en' ? 'ur' : 'en')}
  aria-label={locale === 'en' ? 'Switch to Urdu' : 'Switch to English'}
  aria-pressed={locale === 'ur'}
>
  {locale === 'en' ? 'اردو' : 'English'}
</button>

<div aria-live="polite" className="sr-only">
  {locale === 'ur' ? 'Language changed to Urdu' : 'Language changed to English'}
</div>
```

## Summary

All research tasks completed. No NEEDS CLARIFICATION items remain. Ready for Phase 1 design artifacts.

| Research Area | Decision | Confidence |
|---------------|----------|------------|
| Docusaurus i18n | Built-in i18n with static generation | High |
| RTL Layout | CSS logical properties + dir attribute | High |
| Translation Skill | urdu-translator with term protection | High |
| Glossary Storage | Static JSON + Neon for preferences | High |
| Language Toggle | React Context + localStorage/profile | High |
| Visual Testing | Playwright screenshot comparison | High |
| Cache Invalidation | MD5 content hash | High |
| Accessibility | WCAG 2.1 AA + ARIA live regions | High |
