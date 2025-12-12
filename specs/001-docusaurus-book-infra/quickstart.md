# Quickstart Guide: Docusaurus Book Infrastructure

**Feature**: 002-docusaurus-book-infra
**Audience**: Developers and content authors setting up local development environment
**Time to Complete**: 15-20 minutes

---

## Prerequisites

Before starting, ensure you have:

- **Node.js 18+**: [Download from nodejs.org](https://nodejs.org/)
- **npm 9+** or **yarn 1.22+**: Comes with Node.js
- **Git**: [Download from git-scm.com](https://git-scm.com/)
- **Code Editor**: VS Code (recommended) or any editor with TypeScript support
- **Modern Browser**: Chrome, Firefox, Safari, or Edge (last 2 years)

**Verify Installation**:
```bash
node --version  # Should output v18.x.x or higher
npm --version   # Should output 9.x.x or higher
git --version   # Should output 2.x.x or higher
```

---

## Quick Setup (5 Minutes)

### 1. Clone Repository

```bash
git clone https://github.com/[your-username]/hackathon1_repeat.git
cd hackathon1_repeat
```

### 2. Install Dependencies

```bash
npm install
# or
yarn install
```

**Expected Output**:
```
added 1234 packages in 45s
```

### 3. Start Development Server

```bash
npm run start
# or
yarn start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

### 4. Open in Browser

Navigate to **http://localhost:3000/** - you should see the textbook home page with:
- ✅ Responsive layout
- ✅ Dark mode toggle (top-right corner)
- ✅ Sidebar navigation (collapsible on mobile)
- ✅ Hot reload (changes auto-refresh)

---

## Project Structure

```text
hackathon1_repeat/
├── docs/                       # 📝 Content authors work here
│   ├── intro.md               # Home page content
│   ├── module-1-ros2/         # Example module
│   │   ├── _category_.json    # Module metadata
│   │   └── chapter-1.mdx      # Chapter content
│   └── ...
├── src/                       # 🛠️ Developers work here
│   ├── components/            # Custom React components
│   ├── css/                   # Global styles and theme
│   └── pages/                 # Custom pages (non-doc)
├── static/                    # 🖼️ Static assets (images, fonts)
├── .github/workflows/         # 🚀 CI/CD pipelines
├── docusaurus.config.ts       # ⚙️ Docusaurus configuration
├── sidebars.ts                # 📑 Sidebar structure (optional)
└── package.json               # 📦 Dependencies
```

---

## Common Tasks

### Adding a New Chapter

**For Content Authors (No Coding Required)**:

1. **Create MDX file** in appropriate module directory:
   ```bash
   # Example: Add chapter 3 to module-1
   touch docs/module-1-ros2/chapter-3-services.mdx
   ```

2. **Add frontmatter** at the top of the file:
   ```mdx
   ---
   title: "ROS 2 Services"
   sidebar_position: 3
   description: "Learn how to create ROS 2 services"
   keywords: ["ros2", "services", "tutorial"]
   tags: ["intermediate", "ros2"]
   ---

   # ROS 2 Services

   Your content here...
   ```

3. **Write content** using Markdown + JSX:
   - Headings: `# H1`, `## H2`, `### H3`
   - Code blocks: ` ```python ... ``` `
   - Images: `![Alt text](./image.png)`
   - Links: `[Text](https://example.com)`

4. **Save file** → Browser auto-refreshes with new chapter

### Adding a New Module

1. **Create directory** in `docs/`:
   ```bash
   mkdir docs/module-2-isaac-sim
   ```

2. **Create category file** `docs/module-2-isaac-sim/_category_.json`:
   ```json
   {
     "label": "NVIDIA Isaac Sim",
     "position": 2,
     "collapsible": true,
     "collapsed": false
   }
   ```

3. **Add chapters** as described above

### Customizing Theme Colors

**Edit `src/css/custom.css`**:

```css
/* Light mode colors */
:root {
  --ifm-color-primary: #2e8555;        /* Primary brand color */
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
}

/* Dark mode colors */
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-background-color: #1b1b1d;     /* Dark background */
  --ifm-font-color-base: #e3e3e3;      /* Light text */
}
```

**Save file** → Changes apply immediately in development mode

### Adding Code Syntax Highlighting

**Supported Languages** (configured in `docusaurus.config.ts`):
- JavaScript, TypeScript, JSON (default)
- Python, C++, YAML (added for robotics)

**Usage in MDX**:
````mdx
```python title="publisher.py" showLineNumbers
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```
````

**To add more languages**:

Edit `docusaurus.config.ts`:
```typescript
prism: {
  additionalLanguages: ['python', 'cpp', 'yaml', 'rust'], // Add 'rust'
}
```

---

## Testing

### Run All Tests

```bash
npm run test
# or
yarn test
```

**Test Suites**:
- **Unit Tests**: Jest (components, utilities)
- **Component Tests**: React Testing Library
- **E2E Tests**: Playwright (navigation, responsive, dark mode)
- **Accessibility Tests**: Lighthouse CI

### Run Specific Test Suite

```bash
# Unit tests only
npm run test:unit

# E2E tests only
npm run test:e2e

# Lighthouse CI
npm run test:lighthouse
```

### Manual Testing Checklist

Before committing changes:

- [ ] Dark mode toggle works (click moon/sun icon)
- [ ] Sidebar navigation works on mobile (hamburger menu)
- [ ] New chapters appear in sidebar
- [ ] Code syntax highlighting renders correctly
- [ ] No console errors in browser DevTools
- [ ] Site builds without errors (`npm run build`)

---

## Building for Production

### Build Static Site

```bash
npm run build
# or
yarn build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build" directory
[INFO] Build completed in 2m 34s
```

### Preview Production Build

```bash
npm run serve
# or
yarn serve
```

Navigate to **http://localhost:3000/** to preview production build locally.

---

## Deployment

### Automatic Deployment (GitHub Actions)

**Every commit to `main` branch**:
1. GitHub Actions builds the site
2. Runs tests (Jest, Playwright, Lighthouse)
3. Deploys to GitHub Pages (if tests pass)
4. Site live at `https://[username].github.io/hackathon1_repeat/`

**Check Deployment Status**:
- Go to GitHub repository → **Actions** tab
- View workflow runs and logs

### Manual Deployment (Emergency)

```bash
# Build production site
npm run build

# Deploy to GitHub Pages (requires GH_TOKEN)
GIT_USER=<Your GitHub Username> npm run deploy
```

---

## Troubleshooting

### Build Errors

**Error**: `Cannot find module '@docusaurus/core'`
```bash
# Solution: Reinstall dependencies
rm -rf node_modules package-lock.json
npm install
```

**Error**: `MDX syntax error in chapter-1.mdx`
```bash
# Solution: Check MDX file for:
# - Missing closing backticks in code blocks
# - Unescaped special characters (e.g., < > { })
# - Invalid frontmatter YAML
```

### Hot Reload Not Working

**Solution**:
```bash
# Stop server (Ctrl+C)
# Clear Docusaurus cache
rm -rf .docusaurus
# Restart server
npm run start
```

### Dark Mode Not Persisting

**Issue**: Theme resets to light on page refresh

**Solution**:
- Check browser localStorage is enabled (not in private browsing)
- Clear browser cache and localStorage
- Check Docusaurus config has `respectPrefersColorScheme: true`

### Sidebar Not Updating

**Issue**: New chapters don't appear in sidebar

**Solution**:
1. Check `sidebar_position` in frontmatter (no gaps in sequence)
2. Check `_category_.json` exists in module directory
3. Restart development server
4. Clear `.docusaurus` cache

---

## Development Workflow

### For Content Authors

1. **Pull latest changes**:
   ```bash
   git pull origin main
   ```

2. **Start dev server**:
   ```bash
   npm run start
   ```

3. **Create/edit chapters** in `docs/`:
   - Use MDX syntax
   - Add frontmatter (title, position, tags)
   - Include code examples with syntax highlighting

4. **Preview changes** at `http://localhost:3000/`

5. **Commit changes**:
   ```bash
   git add docs/
   git commit -m "feat: add chapter 3 on ROS 2 services"
   git push origin main
   ```

6. **Verify deployment**:
   - Check GitHub Actions (green checkmark)
   - Visit live site (updates in ~5 minutes)

### For Developers

1. **Create feature branch**:
   ```bash
   git checkout -b feature/custom-component
   ```

2. **Develop in `src/`**:
   - React components: `src/components/`
   - Styles: `src/css/`
   - Custom pages: `src/pages/`

3. **Write tests**:
   - Unit: `src/components/__tests__/`
   - E2E: `tests/e2e/`

4. **Test locally**:
   ```bash
   npm run test
   npm run build
   npm run serve
   ```

5. **Create PR**:
   ```bash
   git push origin feature/custom-component
   # Open PR on GitHub
   ```

6. **Wait for CI**:
   - Tests must pass (Jest, Playwright, Lighthouse)
   - Merge to `main` → Auto-deploy to GitHub Pages

---

## Environment Variables

**Create `.env` file** (for local development only):

```bash
# Not needed for Phase 1 (static site)
# Future phases may require:
# OPENAI_API_KEY=sk-...
# QDRANT_API_KEY=...
```

**Never commit `.env` to Git** (already in `.gitignore`)

---

## VS Code Setup (Recommended)

### Recommended Extensions

Install via VS Code Extensions panel:

- **MDX** (`unifiedjs.vscode-mdx`): MDX syntax highlighting
- **ESLint** (`dbaeumer.vscode-eslint`): JavaScript/TypeScript linting
- **Prettier** (`esbenp.prettier-vscode`): Code formatting
- **TypeScript Vue Plugin** (for TSX support)

### Workspace Settings

Create `.vscode/settings.json`:

```json
{
  "editor.formatOnSave": true,
  "editor.defaultFormatter": "esbenp.prettier-vscode",
  "files.associations": {
    "*.mdx": "mdx"
  },
  "typescript.tsdk": "node_modules/typescript/lib"
}
```

---

## Performance Tips

### Fast Builds

- **Use `--incremental` flag** (only rebuild changed files):
  ```bash
  npm run build -- --incremental
  ```

- **Skip minification in dev**:
  ```bash
  npm run start  # Development mode (no minification)
  ```

### Fast Development

- **Use `--no-open` flag** (don't auto-open browser):
  ```bash
  npm run start -- --no-open
  ```

- **Use `--hot` flag** (hot module replacement):
  ```bash
  npm run start -- --hot
  ```

---

## Next Steps

### For Content Authors
1. ✅ Complete this quickstart
2. 📝 Write first chapter using [MDX Guide](https://mdxjs.com/docs/)
3. 🎨 Customize theme colors (optional)
4. 🚀 Commit and deploy via GitHub

### For Developers
1. ✅ Complete this quickstart
2. 🛠️ Build custom React component (e.g., interactive diagram)
3. 🧪 Write tests (Jest + React Testing Library)
4. 🔍 Run Lighthouse CI locally
5. 📋 Submit PR for review

### For Reviewers
1. ✅ Complete this quickstart
2. 🔎 Review PRs (check tests pass, Lighthouse scores)
3. ✅ Approve and merge
4. 🎉 Verify deployment on live site

---

## Support

**Issues or Questions?**
- **GitHub Issues**: [Report bugs or feature requests](https://github.com/[user]/hackathon1_repeat/issues)
- **Documentation**: See `specs/002-docusaurus-book-infra/` for detailed planning
- **Docusaurus Docs**: [docusaurus.io](https://docusaurus.io/)

**Quality Gates**:
- Lighthouse Performance > 85
- Lighthouse Accessibility > 90
- Build time < 5 minutes
- All tests pass

---

**Version**: 1.0.0
**Last Updated**: 2025-12-11
**Maintained By**: DocusaurusBuilder Agent
