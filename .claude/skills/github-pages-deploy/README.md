# GitHub Pages Deploy Skill

This skill automates the CI/CD pipeline for building and deploying Docusaurus sites to GitHub Pages with proper caching and build optimization.

## Features

- ✅ Automated deployment on push to `main` branch
- ✅ Node.js caching for faster builds
- ✅ Optimized build configuration
- ✅ Proper GitHub Pages setup with permissions
- ✅ Error handling and rollback capabilities
- ✅ Zero downtime deployment

## Prerequisites

1. A Docusaurus project
2. A GitHub repository with GitHub Pages enabled
3. Proper repository permissions

## Usage

### Basic Usage

```bash
skill github-pages-deploy \
  --repo_name "my-docs" \
  --github_org "my-org" \
  --base_url "/my-docs/"
```

### With Custom Configuration

```bash
skill github-pages-deploy \
  --repo_name "documentation" \
  --github_org "company" \
  --base_url "/docs/" \
  --branch "gh-pages" \
  --node_version "20"
```

## Input Parameters

| Parameter | Type | Description | Default | Required |
|-----------|------|-------------|---------|----------|
| `repo_name` | string | The name of the GitHub repository | - | Yes |
| `github_org` | string | The GitHub organization or username | - | Yes |
| `base_url` | string | Base URL for the deployed site | - | Yes |
| `branch` | string | Target branch for deployment | `gh-pages` | No |
| `node_version` | string | Node.js version to use for builds | `20` | No |

## Output Files

The skill creates/modifies the following files:

1. **`.github/workflows/deploy.yml`** - GitHub Actions workflow
2. **`docusaurus.config.ts`** - Docusaurus configuration
3. **`scripts/deploy.js`** - Deployment script

## Configuration Details

### Docusaurus Configuration

The skill automatically configures:

- `url`: `https://{github_org}..github.io`
- `baseUrl`: Your provided base URL
- `organizationName`: GitHub organization
- `projectName`: Repository name
- `trailingSlash`: Set to `false` for clean URLs

### GitHub Actions Workflow

The workflow includes:

- **Trigger**: On push to `main` branch
- **Node.js setup**: With caching for faster builds
- **Build optimization**: Production flags enabled
- **Deployment**: Using GitHub Pages action
- **Permissions**: Properly configured for Pages deployment

### Deployment Script Features

- Automatic git initialization
- Commit history preservation
- Error handling with informative messages
- Rollback capability via git revert

## Quality Criteria

- ✅ Build time < 5 minutes
- ✅ Zero downtime deployment
- ✅ Cache hit rate > 80% on subsequent builds
- ✅ Fail-fast on build errors
- ✅ Automatic rollback on deployment failure

## Branch Protection Recommendation

For production deployments, consider setting up branch protection rules:

1. Go to Repository Settings → Branches
2. Add protection rules for `main` branch:
   - Require pull request reviews before merging
   - Require status checks to pass before merging
   - Require branches to be up to date before merging
   - Include "build" and "deploy" checks as required

## Troubleshooting

### Build Fails

1. Check if your Docusaurus project builds locally: `npm run build`
2. Verify all dependencies are in `package.json`
3. Check the build logs in GitHub Actions

### Deployment Fails

1. Ensure GitHub Pages is enabled in repository settings
2. Verify repository has proper permissions
3. Check that the branch name is correct

### Site Not Loading

1. Verify the `baseUrl` in `docusaurus.config.ts` matches your repository structure
2. Ensure static assets are correctly referenced
3. Check GitHub Pages deployment status

## Example Deployment

After running the skill, your site will be available at:
```
https://{github_org}.github.io/{base_url}
```

For example:
```
https://my-org.github.io/my-docs/
```