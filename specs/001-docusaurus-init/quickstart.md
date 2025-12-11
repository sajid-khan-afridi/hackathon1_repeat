# Quick Start Guide: Book Infrastructure

**Phase**: 1 - Book Infrastructure
**Date**: 2025-12-10
**Feature**: 001-docusaurus-init

## Prerequisites

### System Requirements
- Node.js 18.x or 20.x (Active LTS)
- npm 9.x or yarn 1.22.x
- Git 2.x

### Development Tools (Recommended)
- VS Code with Docusaurus extensions
- Chrome DevTools
- Lighthouse CLI (for performance testing)

## Setup Instructions

### 1. Initialize Repository

```bash
# Clone the repository
git clone <repository-url>
cd hackathon1_repeat

# Switch to feature branch
git checkout 001-docusaurus-init

# Install dependencies
npm install
```

### 2. Local Development

```bash
# Start development server
npm run start

# Open browser to http://localhost:3000
# Site will auto-reload on changes
```

### 3. Build Commands

```bash
# Build for production
npm run build

# Serve built site locally
npm run serve

# Type checking
npm run type-check

# Linting
npm run lint

# Format code
npm run format
```

### 4. Testing

```bash
# Run all tests
npm run test

# Run tests in watch mode
npm run test:watch

# Run Lighthouse audit
npm run lighthouse

# Run accessibility tests
npm run test:a11y
```

## Project Structure

```
hackathon1_repeat/
├── docs/                     # Content source
│   ├── intro.md             # Introduction page
│   ├── getting-started/     # Getting started guides
│   └── modules/             # Learning modules
├── src/                     # Source code
│   ├── components/          # React components
│   ├── css/                 # CSS Modules
│   ├── theme/               # Custom theme components
│   └── pages/               # Custom pages
├── static/                  # Static assets
│   ├── img/                 # Images
│   └── css/                 # Custom CSS
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Sidebar configuration
└── babel.config.js          # Babel configuration
```

## Development Workflow

### 1. Creating New Content

```bash
# Create new markdown file
echo "# New Chapter" > docs/new-chapter.md

# Add front matter
cat > docs/new-chapter.md << EOF
---
title: "New Chapter"
description: "Chapter description"
sidebar_label: "New Chapter"
---

# New Chapter

Content here...
EOF
```

### 2. Adding Custom Components

```bash
# Create new component
mkdir -p src/components/custom
cat > src/components/custom/MyComponent.tsx << EOF
import React from 'react';
import styles from './MyComponent.module.css';

export default function MyComponent() {
  return <div className={styles.container}>My Component</div>;
}
EOF
```

### 3. Customizing Theme

```bash
# Create custom CSS
mkdir -p src/css/custom
cat > src/css/custom.css << EOF
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
}
```

## Configuration

### 1. Docusaurus Configuration (`docusaurus.config.js`)

```javascript
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Interactive learning platform for robotics education',
  url: 'https://your-domain.github.io',
  baseUrl: '/hackathon1_repeat/',

  themeConfig: {
    colorMode: {
      defaultMode: 'system',
      respectPrefersColorScheme: true,
      disableSwitch: false,
    },
    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Robotics Textbook Logo',
        src: 'img/logo.svg',
      },
    },
  },
};
```

### 2. Sidebar Configuration (`sidebars.js`)

```javascript
module.exports = {
  tutorialSidebar: [
    'intro',
    'getting-started/installation',
    'getting-started/quickstart',
    {
      type: 'category',
      label: 'Core Concepts',
      items: [
        'concepts/physical-ai',
        'concepts/humanoid-robotics',
        'concepts/ros2',
      ],
    },
  ],
};
```

## Performance Optimization

### 1. Image Optimization

```bash
# Install ideal image plugin
npm install @docusaurus/plugin-ideal-image

# Add to docusaurus.config.js
plugins: [
  [
    '@docusaurus/plugin-ideal-image',
    {
      quality: 70,
      max: 1030,
      min: 640,
      steps: 2,
      disableInDev: false,
    },
  ],
]
```

### 2. Bundle Analysis

```bash
# Analyze bundle size
npm run build
npx webpack-bundle-analyzer build/static/js/*.js
```

### 3. Lighthouse Testing

```bash
# Run Lighthouse audit
npm install -g lighthouse
lighthouse http://localhost:3000 --output=html --output-path=./lighthouse-report.html
```

## Deployment

### 1. GitHub Pages (Manual)

```bash
# Build and deploy to GitHub Pages
npm run build
npm run deploy

# Or use GitHub Actions (automatic)
git push origin main
```

### 2. Custom Domain Setup

1. Go to repository settings → Pages
2. Add custom domain
3. Configure DNS records
4. Update `baseUrl` in `docusaurus.config.js`

### 3. Environment Variables

```bash
# Create .env.production
NODE_ENV=production
GOOGLE_ANALYTICS_ID=GA-XXXXXXXX-X
```

## Accessibility Testing

### 1. Automated Tests

```bash
# Install axe-core
npm install --save-dev @axe-core/react

# Run accessibility tests
npm run test:a11y
```

### 2. Manual Testing Checklist

- [ ] Keyboard navigation works for all interactive elements
- [ ] Focus indicators are visible
- [ ] Screen reader compatibility (test with NVDA/JAWS)
- [ ] Color contrast ratios meet WCAG 2.1 AA
- [ ] Touch targets are ≥44x44px on mobile

## Troubleshooting

### Common Issues

1. **Build fails with TypeScript errors**
   ```bash
   npm run type-check
   # Fix TypeScript errors in components
   ```

2. **Theme toggle not working**
   - Check localStorage permissions
   - Verify CSS custom properties are defined
   - Test in different browsers

3. **Lighthouse score below 85**
   - Run `npm run analyze-bundle`
   - Optimize images
   - Check bundle size

4. **Responsive layout breaks**
   - Test at exact breakpoints
   - Check CSS media queries
   - Verify mobile-first CSS

### Debug Commands

```bash
# Debug build
DEBUG=docusaurus:* npm run build

# Clear cache
rm -rf .docusaurus build

# Fresh install
rm -rf node_modules package-lock.json
npm install
```

## Next Steps

After completing Phase 1 setup:

1. **Phase 2**: Author initial content (10 chapters)
2. **Phase 3**: Implement RAG chatbot backend
3. **Phase 4A**: Add authentication system
4. **Phase 4B**: Implement personalization
5. **Phase 5**: Add Urdu translation support
6. **Phase 6**: Integration and production deployment

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [React Documentation](https://react.dev)
- [MDN Web Docs](https://developer.mozilla.org)
- [WCAG 2.1 Guidelines](https://www.w3.org/TR/WCAG21/)
- [Lighthouse Performance Audit](https://developers.google.com/web/tools/lighthouse)