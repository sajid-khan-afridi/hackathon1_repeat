# Docusaurus Initialization Skill

This skill initializes a fully configured Docusaurus v3 project with TypeScript, MDX support, and custom theming.

## Usage

```bash
docusaurus-init <project_name> <title> <description> [github_org]
```

### Parameters

- `project_name`: Name of the project directory (required)
- `title`: Title of the documentation site (required)
- `description`: Description of the documentation site (required)
- `github_org`: GitHub organization or username for deployment (optional)

## Example

```bash
# Basic usage
docusaurus-init "my-docs" "My Documentation" "A comprehensive documentation site"

# With GitHub Pages deployment
docusaurus-init "my-docs" "My Documentation" "A comprehensive documentation site" "myorg"
```

## Features

- ✅ Docusaurus v3 with TypeScript
- ✅ MDX support for interactive documentation
- ✅ Custom CSS theming with dark mode
- ✅ Responsive mobile design
- ✅ GitHub Pages deployment configuration
- ✅ 4-module documentation structure
- ✅ Search functionality
- ✅ Code syntax highlighting
- ✅ Table of Contents
- ✅ Pagination

## Generated Structure

```
{project_name}/
├── docusaurus.config.ts     # Main configuration
├── sidebars.ts              # Sidebar configuration
├── package.json             # Dependencies and scripts
├── src/
│   ├── css/
│   │   └── custom.css      # Custom theming
│   └── components/         # Custom components
├── docs/
│   ├── intro.md           # Introduction page
│   ├── getting-started/   # Installation and setup
│   ├── module-1/          # Core concepts
│   ├── module-2/          # Advanced features
│   ├── module-3/          # Integration
│   └── module-4/          # Deployment
├── static/                # Static assets
└── blog/                  # Blog posts
```

## After Initialization

1. Navigate to the project: `cd {project_name}`
2. Start the development server: `npm run start`
3. Open http://localhost:3000 to see your documentation site
4. Start writing documentation in the `docs/` folder

## Quality Assurance

The generated project:

- ✅ Passes `npm run build` without errors
- ✅ Has responsive mobile design
- ✅ Includes dark mode support
- ✅ Follows Docusaurus best practices
- ✅ Includes SEO optimizations
- ✅ Has proper TypeScript configuration