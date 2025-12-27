# Quickstart: Phase 5 Translation

**Feature**: 006-translation
**Date**: 2025-12-27

## Prerequisites

- Node.js 18+ and npm
- Python 3.11+ (for translation scripts)
- Access to `urdu-translator` skill API key
- Existing Docusaurus site with MDX chapters (Phase 2 complete)

## Local Development Setup

### 1. Clone and Install

```bash
# Clone repository (if not already)
git clone https://github.com/your-org/hackathon1_repeat.git
cd hackathon1_repeat

# Install dependencies
npm install

# Install Python dependencies for translation scripts
pip install -r scripts/requirements.txt
```

### 2. Environment Configuration

Create `.env.local` with translation API credentials:

```bash
# Translation API (build-time only)
URDU_TRANSLATOR_API_KEY=your-api-key-here

# Optional: Neon Postgres (for user preferences)
DATABASE_URL=postgres://user:pass@host/db
```

### 3. Generate Translations (Build-Time)

```bash
# Generate Urdu translations for all chapters
npm run translate:build

# Validate technical term preservation
npm run translate:validate

# Generate content hashes for cache invalidation
npm run translate:hash
```

### 4. Run Development Server with Urdu Locale

```bash
# Start Docusaurus with Urdu locale enabled
npm run start -- --locale ur

# Or start with English (default)
npm run start
```

### 5. Test RTL Layout

```bash
# Run visual regression tests
npm run test:visual

# Run component tests for LanguageToggle
npm run test:components -- --grep "LanguageToggle"
```

## Key Commands

| Command | Description |
|---------|-------------|
| `npm run translate:build` | Generate Urdu translations for all chapters |
| `npm run translate:validate` | Validate technical term preservation |
| `npm run translate:hash` | Generate/update content hashes |
| `npm run start -- --locale ur` | Run dev server with Urdu locale |
| `npm run test:visual` | Run Playwright visual regression tests |
| `npm run build` | Build production site with all locales |

## Directory Structure

```text
docs/
├── module-1/
│   └── chapter-1.mdx          # English source
└── i18n/
    └── ur/
        └── docusaurus-plugin-content-docs/
            └── current/
                └── module-1/
                    ├── chapter-1.mdx       # Urdu translation
                    └── chapter-1.meta.json # Cache metadata

src/
├── components/
│   ├── LanguageToggle/        # Language switcher
│   ├── GlossaryTooltip/       # Term hover tooltip
│   └── GlossaryPage/          # Searchable glossary
├── data/
│   └── glossary.json          # Technical term glossary
└── css/
    └── rtl.css                # RTL-specific styles
```

## Testing Translation Locally

### Test Language Toggle

1. Navigate to any chapter page
2. Click the language toggle in the navbar
3. Verify:
   - Content switches to Urdu
   - RTL layout applies (`dir="rtl"`)
   - Technical terms remain in English
   - Code blocks are unchanged

### Test Glossary Tooltip

1. Navigate to a chapter in Urdu mode
2. Hover over a technical term (e.g., "ROS")
3. Verify tooltip shows English term with definition

### Test Visual Regression

```bash
# Update baseline screenshots (first run or after intentional changes)
npm run test:visual:update

# Run comparison tests
npm run test:visual
```

## Troubleshooting

### Translation not appearing

1. Check that `docs/i18n/ur/` directory exists
2. Run `npm run translate:build` to generate translations
3. Restart dev server with `--locale ur` flag

### RTL layout broken

1. Verify `docusaurus.config.js` has `direction: 'rtl'` for `ur` locale
2. Check for hardcoded `left`/`right` CSS values (should use logical properties)
3. Run visual regression tests to identify specific breakages

### Technical terms translated incorrectly

1. Add terms to `src/data/glossary.json`
2. Run `npm run translate:validate` to check preservation
3. Re-run translation with `npm run translate:build --force`

## Next Steps

1. Run `npm run translate:build` to generate initial translations
2. Review 3 sample chapters with native speaker
3. Run full visual regression test suite
4. Commit translated content to repository
