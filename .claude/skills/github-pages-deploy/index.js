#!/usr/bin/env node

import fs from 'fs';
import path from 'path';

/**
 * GitHub Pages Deploy Skill
 * Automates CI/CD pipeline for deploying Docusaurus to GitHub Pages
 */
class GitHubPagesDeploy {
  constructor(options = {}) {
    this.projectRoot = options.projectRoot || process.cwd();
  }

  /**
   * Generate GitHub Actions workflow for deployment
   */
  async generateWorkflow(options = {}) {
    const {
      repoName,
      githubOrg,
      branch = 'gh-pages',
      nodeVersion = '20',
      baseUrl
    } = options;

    if (!repoName || !githubOrg || !baseUrl) {
      throw new Error('repoName, githubOrg, and baseUrl are required');
    }

    const workflowDir = path.join(this.projectRoot, '.github', 'workflows');
    if (!fs.existsSync(workflowDir)) {
      fs.mkdirSync(workflowDir, { recursive: true });
    }

    const workflowContent = `name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '${nodeVersion}'
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build
        env:
          NODE_ENV: production

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: ./build

  deploy:
    environment:
      name: github-pages
      url: \${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
`;

    const workflowPath = path.join(workflowDir, 'deploy.yml');
    fs.writeFileSync(workflowPath, workflowContent, 'utf-8');

    return {
      success: true,
      workflow_file: workflowPath,
      repo_name: repoName,
      github_org: githubOrg
    };
  }

  /**
   * Update docusaurus.config.ts for GitHub Pages deployment
   */
  async updateConfig(options = {}) {
    const {
      repoName,
      githubOrg,
      baseUrl
    } = options;

    if (!repoName || !githubOrg || !baseUrl) {
      throw new Error('repoName, githubOrg, and baseUrl are required');
    }

    const configPath = path.join(this.projectRoot, 'docusaurus.config.ts');

    if (!fs.existsSync(configPath)) {
      throw new Error('docusaurus.config.ts not found');
    }

    let content = fs.readFileSync(configPath, 'utf-8');

    // Update URL and baseUrl
    content = content.replace(
      /url:\s*['"][^'"]*['"]/,
      `url: 'https://${githubOrg}.github.io'`
    );

    content = content.replace(
      /baseUrl:\s*['"][^'"]*['"]/,
      `baseUrl: '${baseUrl}'`
    );

    // Update organizationName and projectName
    content = content.replace(
      /organizationName:\s*['"][^'"]*['"]/,
      `organizationName: '${githubOrg}'`
    );

    content = content.replace(
      /projectName:\s*['"][^'"]*['"]/,
      `projectName: '${repoName}'`
    );

    fs.writeFileSync(configPath, content, 'utf-8');

    return {
      success: true,
      updated_config: configPath
    };
  }

  /**
   * Create deployment script
   */
  async createDeployScript(options = {}) {
    const scriptsDir = path.join(this.projectRoot, 'scripts');
    if (!fs.existsSync(scriptsDir)) {
      fs.mkdirSync(scriptsDir, { recursive: true });
    }

    const scriptContent = `#!/usr/bin/env node

/**
 * Deploy script for GitHub Pages
 * Builds and deploys the Docusaurus site
 */

import { execSync } from 'child_process';

async function deploy() {
  console.log('Building site...');
  execSync('npm run build', { stdio: 'inherit' });

  console.log('Deploying to GitHub Pages...');
  execSync('npm run deploy', { stdio: 'inherit' });

  console.log('Deployment complete!');
}

deploy().catch(error => {
  console.error('Deployment failed:', error);
  process.exit(1);
});
`;

    const scriptPath = path.join(scriptsDir, 'deploy.js');
    fs.writeFileSync(scriptPath, scriptContent, 'utf-8');

    return {
      success: true,
      deploy_script: scriptPath
    };
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: github-pages-deploy <operation> [options]');
    console.error('Operations: workflow, config, script, all');
    console.error('');
    console.error('Examples:');
    console.error('  github-pages-deploy workflow --repo my-book --org myuser --base /my-book/');
    console.error('  github-pages-deploy all --repo my-book --org myuser --base /my-book/');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const deployer = new GitHubPagesDeploy({ projectRoot: process.cwd() });

  try {
    let result;
    const commonOpts = {
      repoName: options.repo,
      githubOrg: options.org,
      baseUrl: options.base,
      branch: options.branch || 'gh-pages',
      nodeVersion: options.node || '20'
    };

    switch (operation) {
      case 'workflow':
        result = await deployer.generateWorkflow(commonOpts);
        break;

      case 'config':
        result = await deployer.updateConfig(commonOpts);
        break;

      case 'script':
        result = await deployer.createDeployScript(commonOpts);
        break;

      case 'all':
        const workflowResult = await deployer.generateWorkflow(commonOpts);
        const configResult = await deployer.updateConfig(commonOpts);
        const scriptResult = await deployer.createDeployScript(commonOpts);
        result = {
          success: true,
          ...workflowResult,
          ...configResult,
          ...scriptResult
        };
        break;

      default:
        console.error('Unknown operation:', operation);
        process.exit(1);
    }

    console.log(JSON.stringify(result, null, 2));
  } catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
  }
}

function parseCliArgs(args) {
  const options = {};
  for (let i = 0; i < args.length; i++) {
    if (args[i].startsWith('--')) {
      const key = args[i].slice(2);
      const value = args[i + 1] && !args[i + 1].startsWith('--') ? args[i + 1] : 'true';
      options[key] = value;
      if (value !== 'true') i++;
    }
  }
  return options;
}

export default GitHubPagesDeploy;
export { GitHubPagesDeploy };

const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('github-pages-deploy/index.js');
if (isMainModule) {
  main().catch(console.error);
}
