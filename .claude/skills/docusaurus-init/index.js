#!/usr/bin/env node

import { execSync } from 'child_process';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

async function createDocusaurusProject(projectName, title, description, githubOrg) {
  console.log(`🚀 Creating Docusaurus project: ${projectName}`);

  // Step 1: Create Docusaurus project
  try {
    execSync(`npx create-docusaurus@latest ${projectName} classic --typescript`, { stdio: 'inherit' });
    console.log('✅ Docusaurus project created successfully');
  } catch (error) {
    console.error('❌ Failed to create Docusaurus project:', error.message);
    process.exit(1);
  }

  const projectPath = path.resolve(projectName);

  // Step 2: Install additional dependencies
  console.log('📦 Installing additional dependencies...');
  try {
    execSync('npm install @docusaurus/plugin-content-docs @docusaurus/theme-mermaid @docusaurus/plugin-ideal-image', {
      cwd: projectPath,
      stdio: 'inherit'
    });
    console.log('✅ Dependencies installed');
  } catch (error) {
    console.error('❌ Failed to install dependencies:', error.message);
  }

  // Step 3: Update docusaurus.config.ts
  console.log('⚙️ Configuring Docusaurus...');
  await updateDocusaurusConfig(projectPath, title, description, githubOrg);

  // Step 4: Update custom.css
  console.log('🎨 Setting up custom theming...');
  await updateCustomCSS(projectPath);

  // Step 5: Update sidebars.ts
  console.log('📚 Setting up sidebar structure...');
  await updateSidebars(projectPath);

  // Step 6: Create initial documentation structure
  console.log('📄 Creating initial documentation...');
  await createInitialDocs(projectPath);

  // Step 7: Add GitHub Pages deployment configuration
  if (githubOrg) {
    console.log('🚀 Setting up GitHub Pages deployment...');
    await setupGitHubPages(projectPath, projectName, githubOrg);
  }

  console.log('\n✅ Docusaurus project initialized successfully!');
  console.log(`\nNext steps:`);
  console.log(`  cd ${projectName}`);
  console.log(`  npm run start`);
  console.log(`\nProject location: ${projectPath}`);

  return projectPath;
}

async function updateDocusaurusConfig(projectPath, title, description, githubOrg) {
  const configPath = path.join(projectPath, 'docusaurus.config.ts');

  const configContent = `import {Config} from '@docusaurus/types';
import {themes} from '@docusaurus/theme-common';
import type {Options as IdealImagePluginOptions} from '@docusaurus/plugin-ideal-image';

const config: Config = {
  title: '${title}',
  tagline: '${description}',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: ${githubOrg ? `'https://${githubOrg}.github.io'` : "'http://localhost:3000'"},
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: ${githubOrg ? `'/${path.basename(projectPath)}/'` : "'/'"},

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: ${githubOrg ? `'${githubOrg}'` : 'null'}, // Usually your GitHub org/user name.
  projectName: '${path.basename(projectPath)}', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.ts'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: ${githubOrg ? `'https://github.com/${githubOrg}/${path.basename(projectPath)}/tree/main/'` : 'null'},
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Useful for generating the sitemap
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } as const,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'api',
        path: 'api',
        routeBasePath: 'api',
        sidebarPath: require.resolve('./sidebars.ts'),
      },
    ],
    [
      '@docusaurus/plugin-ideal-image',
      {
        quality: 70,
        max: 1030, // max resized image's size.
        min: 640, // min resized image's size.
        steps: 2, // the max number of images generated between min and max (inclusive).
        disableInDev: false,
      } as IdealImagePluginOptions,
    ],
  ],

  themeConfig: {
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: '${title}',
      logo: {
        alt: '${title} Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          to: '/blog',
          label: 'Blog',
          position: 'left'
        },
        {
          to: '/api/intro',
          label: 'API',
          position: 'left'
        },
        {
          href: ${githubOrg ? `'https://github.com/${githubOrg}/${path.basename(projectPath)}'` : "'https://github.com/facebook/docusaurus'"},
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Tutorial',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: ${githubOrg ? `'https://github.com/${githubOrg}/${path.basename(projectPath)}'` : "'https://github.com/facebook/docusaurus'"},
            },
          ],
        },
      ],
      copyright: \`Copyright © \${new Date().getFullYear()} ${githubOrg || 'My Project, Inc.'}. Built with Docusaurus.\`,
    },
    prism: {
      theme: themes.github,
      darkTheme: themes.dracula,
      additionalLanguages: ['bash', 'diff', 'json', 'typescript', 'python', 'java', 'go', 'rust'],
    },
  },
};

export default config;
`;

  fs.writeFileSync(configPath, configContent);
}

async function updateCustomCSS(projectPath) {
  const cssPath = path.join(projectPath, 'src', 'css', 'custom.css');

  const cssContent = `/**
 * Any CSS included here will be global. The classic template
 * bundles Infima by default. Infima is a CSS framework designed to
 * work well for content-centric websites.
 */

/* You can override the default Infima variables here. */
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
  --ifm-font-family-base: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans', 'Droid Sans', 'Helvetica Neue', sans-serif;
}

/* Dark theme variables */
[data-theme='dark'] {
  --ifm-color-primary: #4cad80;
  --ifm-color-primary-dark: #479a73;
  --ifm-color-primary-darker: #42916c;
  --ifm-color-primary-darkest: #35775a;
  --ifm-color-primary-light: #54c08b;
  --ifm-color-primary-lighter: #5bc391;
  --ifm-color-primary-lightest: #6dd0a3;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
}

/* Responsive adjustments */
@media only screen and (max-width: 996px) {
  .navbar__toggle {
    display: block;
  }
}

@media only screen and (max-width: 736px) {
  .navbar__title {
    font-size: 1.2rem;
  }

  .footer__link-item {
    font-size: 0.9rem;
  }
}

/* Custom scrollbar for webkit browsers */
::-webkit-scrollbar {
  width: 8px;
  height: 8px;
}

::-webkit-scrollbar-track {
  background: var(--ifm-background-color);
}

::-webkit-scrollbar-thumb {
  background: var(--ifm-color-emphasis-300);
  border-radius: 4px;
}

::-webkit-scrollbar-thumb:hover {
  background: var(--ifm-color-emphasis-400);
}

/* Code block improvements */
.theme-code-block {
  border-radius: 8px;
  overflow: hidden;
}

/* Table of Contents improvements */
.table-of-contents {
  font-size: 0.9rem;
  border-left: 2px solid var(--ifm-color-emphasis-200);
}

.table-of-contents__link {
  border-radius: 4px;
  transition: background-color 0.2s ease;
}

.table-of-contents__link:hover {
  background-color: var(--ifm-color-emphasis-100);
}

/* Admonition improvements */
.admonition {
  border-radius: 8px;
  border: none;
}

/* Mermaid diagram improvements */
.mermaid {
  background-color: var(--ifm-background-surface-color);
  border-radius: 8px;
  padding: 1rem;
}

/* Image improvements */
img {
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

/* Hero section improvements */
.hero__title {
  font-size: 3.5rem;
  font-weight: 700;
  background: linear-gradient(90deg, var(--ifm-color-primary) 0%, var(--ifm-color-secondary) 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero__subtitle {
  font-size: 1.25rem;
  color: var(--ifm-color-emphasis-700);
  max-width: 600px;
  margin: 0 auto;
}

/* Button improvements */
.button {
  border-radius: 8px;
  font-weight: 600;
  transition: all 0.2s ease;
}

.button--primary {
  box-shadow: 0 4px 14px 0 rgba(46, 133, 85, 0.3);
}

.button--primary:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px 0 rgba(46, 133, 85, 0.4);
}

/* Card improvements */
.card {
  border-radius: 12px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.05);
  transition: all 0.2s ease;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
}

/* Feature list improvements */
.features {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  margin: 2rem 0;
}

/* Print styles */
@media print {
  .navbar,
  .footer,
  .pagination-nav {
    display: none !important;
  }

  main {
    max-width: 100% !important;
  }
}
`;

  fs.writeFileSync(cssPath, cssContent);
}

async function updateSidebars(projectPath) {
  const sidebarPath = path.join(projectPath, 'sidebars.ts');

  const sidebarContent = `import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started/installation',
        'getting-started/configuration',
        'getting-started/quick-start',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Core Concepts',
      items: [
        'module-1/introduction',
        'module-1/basic-usage',
        'module-1/advanced-concepts',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Advanced Features',
      items: [
        'module-2/introduction',
        'module-2/customization',
        'module-2/performance',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Integration',
      items: [
        'module-3/introduction',
        'module-3/third-party',
        'module-3/workflows',
      ],
    },
    {
      type: 'category',
    },
    'module-4/deployment',
      ],
    },
  ],

  apiSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Authentication',
      items: [
        'auth/overview',
        'auth/endpoints',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/users',
        'resources/projects',
      ],
    },
  ],
};

export default sidebars;
`;

  fs.writeFileSync(sidebarPath, sidebarContent);
}

async function createInitialDocs(projectPath) {
  const docsBasePath = path.join(projectPath, 'docs');

  // Create module directories
  const modules = ['getting-started', 'module-1', 'module-2', 'module-3', 'module-4'];
  modules.forEach(module => {
    fs.mkdirSync(path.join(docsBasePath, module), { recursive: true });
  });

  // Update intro.md
  const introContent = `---
title: Introduction
sidebar_position: 1
---

# Welcome to ${path.basename(projectPath)}

${path.basename(projectPath)} is a powerful documentation platform built with Docusaurus.

## Features

- 📝 **MDX Support**: Write interactive documentation with JSX components
- 🎨 **Custom Theming**: Beautiful, responsive design with dark mode
- 📚 **Organized Structure**: 4-module documentation layout
- 🔍 **Search**: Full-text search capabilities
- 📱 **Mobile Friendly**: Responsive design for all devices
- ⚡ **Fast**: Optimized for performance

## Quick Start

\`\`\`bash
# Install dependencies
npm install

# Start development server
npm run start

# Build for production
npm run build
\`\`\`

## Documentation Structure

This documentation is organized into the following modules:

1. **Getting Started** - Installation and basic setup
2. **Module 1: Core Concepts** - Fundamental concepts and usage
3. **Module 2: Advanced Features** - Advanced features and customization
4. **Module 3: Integration** - Third-party integrations and workflows
5. **Module 4: Deployment** - Deployment guides and best practices

## Need Help?

- Check out our [FAQ](docs/faq.md)
- Contact support at support@example.com
- Join our [Discord community](https://discord.gg/example)
`;

  fs.writeFileSync(path.join(docsBasePath, 'intro.md'), introContent);

  // Create getting-started files
  const gettingStartedFiles = {
    'installation.md': `---
title: Installation
sidebar_position: 1
---

# Installation

## Prerequisites

- Node.js 16.x or higher
- npm or yarn package manager

## Install Project

\`\`\`bash
# Clone the repository
git clone https://github.com/example/your-project.git
cd your-project

# Install dependencies
npm install
\`\`\`

## Development

\`\`\`bash
# Start development server
npm run start

# Open http://localhost:3000
\`\`\`
`,
    'configuration.md': `---
title: Configuration
sidebar_position: 2
---

# Configuration

## Basic Configuration

\`\`\`ts
// docusaurus.config.ts
const config = {
  title: 'My Site',
  url: 'https://example.com',
  // ... other config
};
\`\`\`

## Environment Variables

Create a \`.env\` file:

\`\`\`env
MY_SITE_URL=https://example.com
API_KEY=your-api-key
\`\`\`
`,
    'quick-start.md': `---
title: Quick Start
sidebar_position: 3
---

# Quick Start

## 5-Minute Setup

1. **Install** dependencies
2. **Configure** your settings
3. **Run** the development server
4. **Build** for production

That's it! You're ready to start documenting.
`
  };

  Object.entries(gettingStartedFiles).forEach(([filename, content]) => {
    fs.writeFileSync(path.join(docsBasePath, 'getting-started', filename), content);
  });

  // Create module placeholders
  for (let i = 1; i <= 4; i++) {
    const moduleIntro = `---
title: Module ${i} Introduction
sidebar_position: 1
---

# Module ${i}: Overview

This module covers important aspects of ${path.basename(projectPath)}.

## Topics

- Topic 1
- Topic 2
- Topic 3

Let's get started!
`;
    fs.writeFileSync(path.join(docsBasePath, `module-${i}`, 'introduction.md'), moduleIntro);
  }
}

async function setupGitHubPages(projectPath, projectName, githubOrg) {
  // Update package.json with deploy script
  const packageJsonPath = path.join(projectPath, 'package.json');
  const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));

  packageJson.scripts.deploy = `USE_SSH=true docusaurus deploy`;

  fs.writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 2));

  // Create .gitignore if it doesn't exist
  const gitignorePath = path.join(projectPath, '.gitignore');
  if (!fs.existsSync(gitignorePath)) {
    const gitignoreContent = `# Dependencies
node_modules/

# Production
/build

# Generated files
.docusaurus
.env.local
.env.production.local

# Debug
npm-debug.log*
yarn-debug.log*
yarn-error.log*

# IDE
.vscode/
.idea/

# OS
.DS_Store
Thumbs.db
`;
    fs.writeFileSync(gitignorePath, gitignoreContent);
  }

  console.log('\n📝 To deploy to GitHub Pages:');
  console.log(`1. Create a new repository at https://github.com/${githubOrg}/${projectName}`);
  console.log(`2. Add remote: git remote add origin https://github.com/${githubOrg}/${projectName}.git`);
  console.log(`3. Push: git push -u origin main`);
  console.log(`4. Run: npm run deploy`);
}

// Parse command line arguments
const args = process.argv.slice(2);
if (args.length < 3) {
  console.error('Usage: docusaurus-init <project_name> <title> <description> [github_org]');
  process.exit(1);
}

const [projectName, title, description, githubOrg] = args;

// Run the initialization
createDocusaurusProject(projectName, title, description, githubOrg)
  .then((projectPath) => {
    console.log('\n✨ Project successfully created at:', projectPath);
  })
  .catch((error) => {
    console.error('❌ Error creating project:', error);
    process.exit(1);
  });