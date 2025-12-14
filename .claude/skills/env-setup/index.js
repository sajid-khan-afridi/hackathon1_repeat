#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import crypto from 'crypto';

/**
 * Environment Setup Skill
 * Generates and validates .env files with all required environment variables
 */
class EnvSetup {
  constructor(options = {}) {
    this.projectRoot = options.projectRoot || process.cwd();

    // Define required variables with their metadata
    this.requiredVariables = {
      DATABASE_URL: {
        description: 'Neon PostgreSQL connection string',
        format: 'postgres://user:password@host/database',
        required: true,
        example: 'postgres://user:pass@ep-cool-darkness-123456.us-east-2.aws.neon.tech/neondb',
        validate: (value) => value.startsWith('postgres://') || value.startsWith('postgresql://')
      },
      QDRANT_URL: {
        description: 'Qdrant vector database URL',
        format: 'URL',
        required: true,
        example: 'http://localhost:6333',
        validate: (value) => value.startsWith('http://') || value.startsWith('https://')
      },
      QDRANT_API_KEY: {
        description: 'Qdrant API key (optional for local, required for cloud)',
        format: 'string',
        required: false,
        example: 'your-qdrant-api-key',
        validate: () => true
      },
      OPENAI_API_KEY: {
        description: 'OpenAI API key for embeddings and chat completions',
        format: 'sk-...',
        required: true,
        example: 'sk-proj-xxxxxxxxxxxxxxxxxxxxx',
        validate: (value) => value.startsWith('sk-')
      },
      AUTH_SECRET: {
        description: 'Secret key for JWT token signing (32+ characters)',
        format: 'base64',
        required: true,
        example: 'Run: openssl rand -base64 32',
        validate: (value) => value.length >= 32
      },
      AUTH_URL: {
        description: 'Base URL for authentication callbacks',
        format: 'URL',
        required: true,
        example: 'http://localhost:3000',
        validate: (value) => value.startsWith('http://') || value.startsWith('https://')
      },
      DATABASE_POOL_MAX: {
        description: 'Maximum database connection pool size',
        format: 'integer',
        required: false,
        default: '10',
        example: '10',
        validate: (value) => !isNaN(parseInt(value))
      },
      EMBEDDING_MODEL: {
        description: 'OpenAI embedding model to use',
        format: 'string',
        required: false,
        default: 'text-embedding-3-small',
        example: 'text-embedding-3-small',
        validate: () => true
      },
      NODE_ENV: {
        description: 'Node environment',
        format: 'string',
        required: false,
        default: 'development',
        example: 'development',
        validate: (value) => ['development', 'production', 'test'].includes(value)
      }
    };
  }

  /**
   * Generate a new .env file
   */
  async generate(options = {}) {
    const {
      environment = 'development',
      outputPath = '.env',
      overwrite = false,
      includeComments = true
    } = options;

    const fullPath = path.resolve(this.projectRoot, outputPath);

    // Check if file exists
    if (fs.existsSync(fullPath) && !overwrite) {
      throw new Error(`File ${outputPath} already exists. Use overwrite: true to replace.`);
    }

    // Generate content
    const content = this.generateEnvContent(environment, includeComments);

    // Write file
    fs.writeFileSync(fullPath, content, 'utf-8');

    return {
      success: true,
      env_file_path: fullPath,
      variables_count: Object.keys(this.requiredVariables).length
    };
  }

  /**
   * Generate .env.example template
   */
  async template(options = {}) {
    const {
      outputPath = '.env.example',
      includeComments = true
    } = options;

    const fullPath = path.resolve(this.projectRoot, outputPath);

    const content = this.generateTemplateContent(includeComments);

    fs.writeFileSync(fullPath, content, 'utf-8');

    return {
      success: true,
      template_path: fullPath
    };
  }

  /**
   * Validate an existing .env file
   */
  async validate(options = {}) {
    const { envPath = '.env' } = options;
    const fullPath = path.resolve(this.projectRoot, envPath);

    if (!fs.existsSync(fullPath)) {
      return {
        valid: false,
        missing_vars: Object.keys(this.requiredVariables).filter(
          key => this.requiredVariables[key].required
        ),
        invalid_values: [],
        error: `File ${envPath} does not exist`
      };
    }

    const content = fs.readFileSync(fullPath, 'utf-8');
    const envVars = this.parseEnvFile(content);

    const missingVars = [];
    const invalidValues = [];

    for (const [key, config] of Object.entries(this.requiredVariables)) {
      const value = envVars[key];

      if (config.required && !value) {
        missingVars.push(key);
      } else if (value && config.validate && !config.validate(value)) {
        invalidValues.push({
          key,
          value: value.substring(0, 10) + '...',
          expected: config.format
        });
      }
    }

    return {
      valid: missingVars.length === 0 && invalidValues.length === 0,
      missing_vars: missingVars,
      invalid_values: invalidValues,
      env_file_path: fullPath
    };
  }

  /**
   * Update an existing .env file with new variables
   */
  async update(options = {}) {
    const {
      envPath = '.env',
      variables = {}
    } = options;

    const fullPath = path.resolve(this.projectRoot, envPath);

    let existingVars = {};
    if (fs.existsSync(fullPath)) {
      const content = fs.readFileSync(fullPath, 'utf-8');
      existingVars = this.parseEnvFile(content);
    }

    // Merge variables
    const mergedVars = { ...existingVars, ...variables };

    // Generate new content
    const lines = [];
    for (const [key, value] of Object.entries(mergedVars)) {
      if (this.requiredVariables[key]) {
        lines.push(`# ${this.requiredVariables[key].description}`);
      }
      lines.push(`${key}=${value}`);
      lines.push('');
    }

    fs.writeFileSync(fullPath, lines.join('\n'), 'utf-8');

    return {
      success: true,
      env_file_path: fullPath,
      updated_variables: Object.keys(variables)
    };
  }

  /**
   * Generate .env file content
   */
  generateEnvContent(environment, includeComments) {
    const lines = [];

    if (includeComments) {
      lines.push('# ============================================');
      lines.push(`# Environment: ${environment}`);
      lines.push(`# Generated: ${new Date().toISOString()}`);
      lines.push('# ============================================');
      lines.push('');
    }

    for (const [key, config] of Object.entries(this.requiredVariables)) {
      if (includeComments) {
        lines.push(`# ${config.description}`);
        if (config.format) {
          lines.push(`# Format: ${config.format}`);
        }
      }

      let value = '';
      if (key === 'AUTH_SECRET') {
        // Generate a random secret
        value = crypto.randomBytes(32).toString('base64');
      } else if (key === 'NODE_ENV') {
        value = environment;
      } else if (config.default) {
        value = config.default;
      } else {
        value = config.example || '';
      }

      lines.push(`${key}=${value}`);
      lines.push('');
    }

    return lines.join('\n');
  }

  /**
   * Generate .env.example template content
   */
  generateTemplateContent(includeComments) {
    const lines = [];

    if (includeComments) {
      lines.push('# ============================================');
      lines.push('# Environment Variables Template');
      lines.push('# Copy this file to .env and fill in your values');
      lines.push('# ============================================');
      lines.push('');
    }

    for (const [key, config] of Object.entries(this.requiredVariables)) {
      if (includeComments) {
        lines.push(`# ${config.description}`);
        lines.push(`# Required: ${config.required ? 'Yes' : 'No'}`);
        if (config.format) {
          lines.push(`# Format: ${config.format}`);
        }
      }

      const value = config.required ? '' : (config.default || '');
      lines.push(`${key}=${value}`);
      lines.push('');
    }

    return lines.join('\n');
  }

  /**
   * Parse .env file content into key-value pairs
   */
  parseEnvFile(content) {
    const vars = {};
    const lines = content.split('\n');

    for (const line of lines) {
      const trimmed = line.trim();

      // Skip comments and empty lines
      if (!trimmed || trimmed.startsWith('#')) {
        continue;
      }

      const eqIndex = trimmed.indexOf('=');
      if (eqIndex > 0) {
        const key = trimmed.substring(0, eqIndex).trim();
        let value = trimmed.substring(eqIndex + 1).trim();

        // Remove quotes if present
        if ((value.startsWith('"') && value.endsWith('"')) ||
            (value.startsWith("'") && value.endsWith("'"))) {
          value = value.slice(1, -1);
        }

        vars[key] = value;
      }
    }

    return vars;
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: env-setup <operation> [options]');
    console.error('Operations: generate, validate, update, template');
    console.error('');
    console.error('Examples:');
    console.error('  env-setup generate --env development --output .env');
    console.error('  env-setup validate --path .env');
    console.error('  env-setup template --output .env.example');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const envSetup = new EnvSetup({ projectRoot: process.cwd() });

  try {
    let result;
    switch (operation) {
      case 'generate':
        result = await envSetup.generate({
          environment: options.env || 'development',
          outputPath: options.output || '.env',
          overwrite: options.overwrite === 'true',
          includeComments: options.comments !== 'false'
        });
        console.log('Generated .env file:', result.env_file_path);
        break;

      case 'validate':
        result = await envSetup.validate({
          envPath: options.path || '.env'
        });
        if (result.valid) {
          console.log('Validation passed!');
        } else {
          console.log('Validation failed:');
          if (result.missing_vars.length > 0) {
            console.log('  Missing variables:', result.missing_vars.join(', '));
          }
          if (result.invalid_values.length > 0) {
            console.log('  Invalid values:', result.invalid_values.map(v => v.key).join(', '));
          }
          process.exit(1);
        }
        break;

      case 'update':
        const variables = {};
        for (const [key, value] of Object.entries(options)) {
          if (!['path'].includes(key)) {
            variables[key.toUpperCase()] = value;
          }
        }
        result = await envSetup.update({
          envPath: options.path || '.env',
          variables
        });
        console.log('Updated .env file:', result.env_file_path);
        break;

      case 'template':
        result = await envSetup.template({
          outputPath: options.output || '.env.example',
          includeComments: options.comments !== 'false'
        });
        console.log('Generated template:', result.template_path);
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

/**
 * Parse CLI arguments into options object
 */
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

// Export for use as a module
export default EnvSetup;
export { EnvSetup };

// Run CLI if called directly
const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('env-setup/index.js');
if (isMainModule) {
  main().catch(console.error);
}
