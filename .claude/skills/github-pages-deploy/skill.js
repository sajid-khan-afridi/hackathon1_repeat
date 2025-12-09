const fs = require('fs')
const path = require('path')
const { execSync } = require('child_process')

/**
 * GitHub Pages Deploy Skill for Docusaurus
 */
async function execute(inputs) {
  const { repo_name, github_org, branch = 'gh-pages', node_version = '20', base_url } = inputs

  // Validate required inputs
  if (!repo_name || !github_org || !base_url) {
    throw new Error('Missing required inputs: repo_name, github_org, and base_url are required')
  }

  console.log('🚀 Setting up GitHub Pages deployment for Docusaurus...')
  console.log(`Repository: ${github_org}/${repo_name}`)
  console.log(`Base URL: ${base_url}`)
  console.log(`Node.js version: ${node_version}`)

  try {
    // Create .github/workflows directory
    const workflowsDir = path.join(process.cwd(), '.github', 'workflows')
    if (!fs.existsSync(workflowsDir)) {
      fs.mkdirSync(workflowsDir, { recursive: true })
    }

    // Create scripts directory
    const scriptsDir = path.join(process.cwd(), 'scripts')
    if (!fs.existsSync(scriptsDir)) {
      fs.mkdirSync(scriptsDir, { recursive: true })
    }

    // Read and process GitHub Actions workflow template
    console.log('📝 Creating GitHub Actions workflow...')
    const workflowTemplate = fs.readFileSync(
      path.join(__dirname, 'templates', 'deploy.yml.hbs'),
      'utf8'
    )

    const workflowContent = workflowTemplate
      .replace(/\{\{repo_name\}\}/g, repo_name)
      .replace(/\{\{github_org\}\}/g, github_org)
      .replace(/\{\{branch\}\}/g, branch)
      .replace(/\{\{node_version\}\}/g, node_version)
      .replace(/\{\{base_url\}\}/g, base_url)

    fs.writeFileSync(
      path.join(workflowsDir, 'deploy.yml'),
      workflowContent
    )

    // Read and process Docusaurus configuration template
    console.log('⚙️ Updating Docusaurus configuration...')
    const configTemplate = fs.readFileSync(
      path.join(__dirname, 'templates', 'docusaurus.config.ts.hbs'),
      'utf8'
    )

    const configContent = configTemplate
      .replace(/\{\{repo_name\}\}/g, repo_name)
      .replace(/\{\{github_org\}\}/g, github_org)
      .replace(/\{\{base_url\}\}/g, base_url)

    fs.writeFileSync(
      path.join(process.cwd(), 'docusaurus.config.ts'),
      configContent
    )

    // Read and process deployment script
    console.log('📜 Creating deployment script...')
    const scriptTemplate = fs.readFileSync(
      path.join(__dirname, 'templates', 'deploy.js.hbs'),
      'utf8'
    )

    const scriptContent = scriptTemplate
      .replace(/\{\{repo_name\}\}/g, repo_name)
      .replace(/\{\{github_org\}\}/g, github_org)
      .replace(/\{\{branch\}\}/g, branch)
      .replace(/\{\{base_url\}\}/g, base_url)

    fs.writeFileSync(
      path.join(scriptsDir, 'deploy.js'),
      scriptContent
    )

    // Make deploy script executable (Unix systems)
    if (process.platform !== 'win32') {
      try {
        execSync(`chmod +x ${path.join(scriptsDir, 'deploy.js')}`)
      } catch (error) {
        // Continue even if chmod fails
      }
    }

    // Create package.json script if it doesn't exist
    const packageJsonPath = path.join(process.cwd(), 'package.json')
    if (fs.existsSync(packageJsonPath)) {
      const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'))

      if (!packageJson.scripts) {
        packageJson.scripts = {}
      }

      if (!packageJson.scripts['deploy']) {
        packageJson.scripts['deploy'] = 'node scripts/deploy.js'
        fs.writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 2))
        console.log('📦 Added deploy script to package.json')
      }
    }

    console.log('\n✅ GitHub Pages deployment setup complete!')
    console.log('\n📋 Next steps:')
    console.log(`1. Commit and push your changes to the main branch`)
    console.log(`2. Enable GitHub Pages in your repository settings`)
    console.log(`3. Select the 'gh-pages' branch as the source`)
    console.log(`4. The workflow will automatically deploy on each push to main`)
    console.log(`\n📖 Your site will be available at: https://${github_org}.github.io${base_url}`)

    // Additional recommendations
    console.log('\n🔧 Additional recommendations:')
    console.log('- Set up branch protection rules for the main branch')
    console.log('- Configure GitHub Pages custom domain if needed')
    console.log('- Monitor build times in the Actions tab')
    console.log('- Set up status checks in your PR template')

    return {
      success: true,
      files_created: [
        '.github/workflows/deploy.yml',
        'docusaurus.config.ts',
        'scripts/deploy.js'
      ],
      deployment_url: `https://${github_org}.github.io${base_url}`
    }

  } catch (error) {
    console.error('❌ Error setting up GitHub Pages deployment:', error.message)
    throw error
  }
}

module.exports = { execute }