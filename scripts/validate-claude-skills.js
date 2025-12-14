#!/usr/bin/env node

/**
 * Claude Skills Validation Script
 *
 * Validates that all skills in .claude/skills/ have the required files:
 * - skill.json (metadata and configuration)
 * - README.md (documentation)
 * - index.js or index.ts (JavaScript/TypeScript entry point)
 *   OR tools/*.py (Python entry point for Python-based skills)
 *
 * Usage:
 *   node scripts/validate-claude-skills.js [--fix] [--verbose]
 *
 * Options:
 *   --fix      Attempt to create missing documentation from skill.json
 *   --verbose  Show detailed information about each skill
 *
 * Exit codes:
 *   0 - All skills valid
 *   1 - Validation errors found
 */

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '..');
const skillsDir = path.join(projectRoot, '.claude', 'skills');

// Required files for each skill
const REQUIRED_FILES = {
  metadata: 'skill.json',
  documentation: 'README.md'  // Standardized on README.md
};

// Valid entry point patterns
const VALID_ENTRY_POINTS = [
  'index.js',
  'index.ts',
  'index.mjs',
  'index.cjs'
];

// Python-based skills can have tools/*.py as entry point
const PYTHON_ENTRY_PATTERNS = [
  'tools/*.py',
  '*.py'
];

class SkillValidator {
  constructor(options = {}) {
    this.verbose = options.verbose || false;
    this.fix = options.fix || false;
    this.errors = [];
    this.warnings = [];
    this.skills = [];
  }

  /**
   * Validate all skills
   */
  async validate() {
    console.log('Validating Claude Code skills...\n');

    if (!fs.existsSync(skillsDir)) {
      this.errors.push(`Skills directory not found: ${skillsDir}`);
      return this.report();
    }

    // Get all skill directories (excluding shared/)
    const entries = fs.readdirSync(skillsDir, { withFileTypes: true });
    const skillDirs = entries
      .filter(e => e.isDirectory() && e.name !== 'shared' && e.name !== 'node_modules')
      .map(e => e.name);

    console.log(`Found ${skillDirs.length} skills to validate\n`);

    for (const skillName of skillDirs) {
      await this.validateSkill(skillName);
    }

    return this.report();
  }

  /**
   * Validate a single skill
   */
  async validateSkill(skillName) {
    const skillPath = path.join(skillsDir, skillName);
    const skill = {
      name: skillName,
      path: skillPath,
      hasMetadata: false,
      hasDocumentation: false,
      hasEntryPoint: false,
      entryPointType: null,
      errors: [],
      warnings: []
    };

    if (this.verbose) {
      console.log(`Checking: ${skillName}`);
    }

    // Check for skill.json
    const metadataPath = path.join(skillPath, REQUIRED_FILES.metadata);
    if (fs.existsSync(metadataPath)) {
      skill.hasMetadata = true;
      try {
        const metadata = JSON.parse(fs.readFileSync(metadataPath, 'utf-8'));
        skill.metadata = metadata;
      } catch (e) {
        skill.errors.push(`Invalid JSON in skill.json: ${e.message}`);
      }
    } else {
      skill.errors.push(`Missing required file: ${REQUIRED_FILES.metadata}`);
    }

    // Check for README.md (documentation)
    const docPath = path.join(skillPath, REQUIRED_FILES.documentation);
    if (fs.existsSync(docPath)) {
      skill.hasDocumentation = true;
    } else {
      // Check for legacy SKILL.md
      const legacyDocPath = path.join(skillPath, 'SKILL.md');
      if (fs.existsSync(legacyDocPath)) {
        skill.hasDocumentation = true;
        skill.warnings.push('Using SKILL.md instead of README.md (consider renaming for consistency)');
      } else {
        skill.errors.push(`Missing required file: ${REQUIRED_FILES.documentation}`);

        if (this.fix && skill.metadata) {
          this.generateReadme(skillPath, skill.metadata);
          skill.hasDocumentation = true;
          skill.errors.pop(); // Remove the error we just fixed
          skill.warnings.push('Generated README.md from skill.json metadata');
        }
      }
    }

    // Check for entry point
    for (const entryPoint of VALID_ENTRY_POINTS) {
      const entryPath = path.join(skillPath, entryPoint);
      if (fs.existsSync(entryPath)) {
        skill.hasEntryPoint = true;
        skill.entryPointType = 'javascript';
        break;
      }
    }

    // Check for Python entry point if no JS entry found
    if (!skill.hasEntryPoint) {
      const toolsDir = path.join(skillPath, 'tools');
      if (fs.existsSync(toolsDir)) {
        const pyFiles = fs.readdirSync(toolsDir).filter(f => f.endsWith('.py'));
        if (pyFiles.length > 0) {
          skill.hasEntryPoint = true;
          skill.entryPointType = 'python';
        }
      }

      // Also check for root-level .py files
      if (!skill.hasEntryPoint) {
        const rootPyFiles = fs.readdirSync(skillPath).filter(f => f.endsWith('.py') && !f.startsWith('test'));
        if (rootPyFiles.length > 0) {
          skill.hasEntryPoint = true;
          skill.entryPointType = 'python';
        }
      }
    }

    if (!skill.hasEntryPoint) {
      skill.errors.push('Missing entry point (index.js, index.ts, or tools/*.py)');
    }

    // Collect results
    this.skills.push(skill);
    this.errors.push(...skill.errors.map(e => `${skillName}: ${e}`));
    this.warnings.push(...skill.warnings.map(w => `${skillName}: ${w}`));

    // Print status
    const status = skill.errors.length === 0 ? 'PASS' : 'FAIL';
    const statusIcon = skill.errors.length === 0 ? '\u2713' : '\u2717';

    if (this.verbose || skill.errors.length > 0) {
      console.log(`  ${statusIcon} ${skillName}: ${status}`);
      if (skill.errors.length > 0) {
        skill.errors.forEach(e => console.log(`    - ${e}`));
      }
      if (skill.warnings.length > 0 && this.verbose) {
        skill.warnings.forEach(w => console.log(`    [warn] ${w}`));
      }
    }
  }

  /**
   * Generate README.md from skill.json metadata
   */
  generateReadme(skillPath, metadata) {
    const readme = `# ${metadata.name}

${metadata.description || 'A Claude Code skill.'}

## Overview

${metadata.type ? `Type: ${metadata.type}` : ''}

## Parameters

${this.formatParameters(metadata.parameters)}

## Outputs

${this.formatOutputs(metadata.outputs)}

## Environment Variables

${this.formatEnvironment(metadata.environment)}

## Dependencies

${metadata.dependencies ? metadata.dependencies.map(d => `- ${d}`).join('\n') : 'None'}

## Tags

${metadata.tags ? metadata.tags.map(t => `\`${t}\``).join(' ') : 'None'}

---
*Generated from skill.json*
`;

    const readmePath = path.join(skillPath, 'README.md');
    fs.writeFileSync(readmePath, readme, 'utf-8');
    console.log(`  Generated: ${readmePath}`);
  }

  formatParameters(params) {
    if (!params || Object.keys(params).length === 0) {
      return 'None';
    }

    const lines = ['| Parameter | Type | Required | Description |',
                   '|-----------|------|----------|-------------|'];

    for (const [name, config] of Object.entries(params)) {
      const type = config.type || 'string';
      const required = config.required ? 'Yes' : 'No';
      const desc = config.description || '-';
      lines.push(`| ${name} | ${type} | ${required} | ${desc} |`);
    }

    return lines.join('\n');
  }

  formatOutputs(outputs) {
    if (!outputs || Object.keys(outputs).length === 0) {
      return 'None';
    }

    const lines = ['| Output | Type | Description |',
                   '|--------|------|-------------|'];

    for (const [name, config] of Object.entries(outputs)) {
      const type = config.type || 'any';
      const desc = config.description || '-';
      lines.push(`| ${name} | ${type} | ${desc} |`);
    }

    return lines.join('\n');
  }

  formatEnvironment(env) {
    if (!env || Object.keys(env).length === 0) {
      return 'None required';
    }

    const lines = ['| Variable | Required | Description |',
                   '|----------|----------|-------------|'];

    for (const [name, config] of Object.entries(env)) {
      const required = config.required ? 'Yes' : 'No';
      const desc = config.description || '-';
      lines.push(`| ${name} | ${required} | ${desc} |`);
    }

    return lines.join('\n');
  }

  /**
   * Generate validation report
   */
  report() {
    console.log('\n' + '='.repeat(50));
    console.log('Validation Summary');
    console.log('='.repeat(50));

    const validSkills = this.skills.filter(s => s.errors.length === 0);
    const invalidSkills = this.skills.filter(s => s.errors.length > 0);

    console.log(`\nTotal skills: ${this.skills.length}`);
    console.log(`  Valid: ${validSkills.length}`);
    console.log(`  Invalid: ${invalidSkills.length}`);

    if (this.warnings.length > 0) {
      console.log(`\nWarnings: ${this.warnings.length}`);
      if (this.verbose) {
        this.warnings.forEach(w => console.log(`  - ${w}`));
      }
    }

    if (invalidSkills.length > 0) {
      console.log('\nInvalid skills:');
      invalidSkills.forEach(s => {
        console.log(`  - ${s.name}`);
        s.errors.forEach(e => console.log(`      ${e}`));
      });
      console.log('\nValidation FAILED');
      return 1;
    }

    console.log('\nValidation PASSED');
    return 0;
  }
}

/**
 * Main entry point
 */
async function main() {
  const args = process.argv.slice(2);
  const options = {
    verbose: args.includes('--verbose') || args.includes('-v'),
    fix: args.includes('--fix')
  };

  if (args.includes('--help') || args.includes('-h')) {
    console.log(`
Claude Skills Validation Script

Usage: node scripts/validate-claude-skills.js [options]

Options:
  --fix      Attempt to create missing documentation from skill.json
  --verbose  Show detailed information about each skill
  --help     Show this help message

Required files for each skill:
  - skill.json   Metadata and configuration
  - README.md    Documentation
  - index.js     Entry point (or index.ts, tools/*.py)

Exit codes:
  0 - All skills valid
  1 - Validation errors found
`);
    process.exit(0);
  }

  const validator = new SkillValidator(options);
  const exitCode = await validator.validate();
  process.exit(exitCode);
}

main().catch(error => {
  console.error('Validation failed:', error);
  process.exit(1);
});
