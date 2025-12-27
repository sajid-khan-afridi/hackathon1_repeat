#!/usr/bin/env ts-node
/**
 * Technical Term Validation Script
 * Validates that technical terms are preserved in translated content
 * Phase 5: Translation Feature - T010
 *
 * Usage:
 *   npx ts-node scripts/validate-terms.ts [options]
 *
 * Options:
 *   --source, -s    Source file or directory (required)
 *   --target, -t    Target (translated) file or directory (required)
 *   --strict        Fail on any missing term (default: warn)
 *   --json          Output as JSON
 */

import * as fs from 'fs';
import * as path from 'path';

interface TermDefinition {
  term: string;
  category: string;
  translationStatus: string;
}

interface ValidationResult {
  file: string;
  totalTerms: number;
  preservedTerms: number;
  missingTerms: string[];
  preservationRate: number;
  passed: boolean;
}

interface ValidationReport {
  generated: string;
  strictMode: boolean;
  results: ValidationResult[];
  summary: {
    totalFiles: number;
    passedFiles: number;
    totalTermsChecked: number;
    totalTermsPreserved: number;
    overallPreservationRate: number;
    allPassed: boolean;
  };
}

const TERMS_FILE = path.join(process.cwd(), 'src', 'data', 'technical-terms.json');
const MIN_PRESERVATION_RATE = 1.0; // 100% required per SC-004

/**
 * Load technical terms from JSON file
 */
function loadTerms(): TermDefinition[] {
  if (!fs.existsSync(TERMS_FILE)) {
    console.error(`Error: Technical terms file not found at ${TERMS_FILE}`);
    process.exit(1);
  }

  const data = JSON.parse(fs.readFileSync(TERMS_FILE, 'utf8'));
  return data.terms.filter((t: TermDefinition) => t.translationStatus === 'preserve');
}

/**
 * Find terms that should be preserved in content
 */
function findPreservableTerms(content: string, terms: TermDefinition[]): string[] {
  const found: string[] = [];

  for (const term of terms) {
    // Create regex that matches the term as a whole word (case-insensitive)
    const regex = new RegExp(`\\b${escapeRegex(term.term)}\\b`, 'gi');
    if (regex.test(content)) {
      found.push(term.term);
    }
  }

  return [...new Set(found)]; // Remove duplicates
}

/**
 * Escape special regex characters
 */
function escapeRegex(str: string): string {
  return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

/**
 * Check if a term is preserved in translated content
 */
function isTermPreserved(term: string, content: string): boolean {
  const regex = new RegExp(`\\b${escapeRegex(term)}\\b`, 'gi');
  return regex.test(content);
}

/**
 * Validate a single file pair
 */
function validateFilePair(
  sourceFile: string,
  targetFile: string,
  terms: TermDefinition[]
): ValidationResult {
  const sourceContent = fs.readFileSync(sourceFile, 'utf8');
  const targetContent = fs.existsSync(targetFile)
    ? fs.readFileSync(targetFile, 'utf8')
    : '';

  // Find terms in source that should be preserved
  const termsInSource = findPreservableTerms(sourceContent, terms);

  // Check which terms are preserved in target
  const preservedTerms: string[] = [];
  const missingTerms: string[] = [];

  for (const term of termsInSource) {
    if (isTermPreserved(term, targetContent)) {
      preservedTerms.push(term);
    } else {
      missingTerms.push(term);
    }
  }

  const preservationRate = termsInSource.length > 0
    ? preservedTerms.length / termsInSource.length
    : 1.0;

  return {
    file: path.relative(process.cwd(), sourceFile).replace(/\\/g, '/'),
    totalTerms: termsInSource.length,
    preservedTerms: preservedTerms.length,
    missingTerms,
    preservationRate,
    passed: preservationRate >= MIN_PRESERVATION_RATE,
  };
}

/**
 * Find matching target file for source file
 */
function findTargetFile(sourceFile: string, targetDir: string): string {
  const relativePath = path.relative(
    path.join(process.cwd(), 'docs'),
    sourceFile
  );
  return path.join(targetDir, relativePath);
}

/**
 * Get all MDX files in a directory
 */
function getMdxFiles(dir: string): string[] {
  const files: string[] = [];

  function walkDir(currentDir: string): void {
    const entries = fs.readdirSync(currentDir, { withFileTypes: true });
    for (const entry of entries) {
      const fullPath = path.join(currentDir, entry.name);
      if (entry.isDirectory() && !entry.name.startsWith('.') && entry.name !== 'i18n') {
        walkDir(fullPath);
      } else if (entry.isFile() && /\.mdx?$/.test(entry.name)) {
        files.push(fullPath);
      }
    }
  }

  if (fs.statSync(dir).isDirectory()) {
    walkDir(dir);
  } else {
    files.push(dir);
  }

  return files;
}

/**
 * Parse command line arguments
 */
function parseArgs(): { source: string; target: string; strict: boolean; json: boolean } {
  const args = process.argv.slice(2);

  const sourceIndex = args.findIndex((a) => a === '-s' || a === '--source');
  const targetIndex = args.findIndex((a) => a === '-t' || a === '--target');
  const strict = args.includes('--strict');
  const json = args.includes('--json');

  const source = sourceIndex !== -1 ? args[sourceIndex + 1] : '';
  const target = targetIndex !== -1 ? args[targetIndex + 1] : '';

  if (!source || !target) {
    console.error('Usage: validate-terms.ts -s <source> -t <target> [--strict] [--json]');
    process.exit(1);
  }

  return { source, target, strict, json };
}

/**
 * Main function
 */
function main(): void {
  const { source, target, strict, json } = parseArgs();

  // Load terms
  const terms = loadTerms();
  console.error(`Loaded ${terms.length} preservable technical terms`);

  // Get source files
  const sourceFiles = getMdxFiles(source);
  console.error(`Found ${sourceFiles.length} source files to validate`);

  // Validate each file
  const results: ValidationResult[] = [];

  for (const sourceFile of sourceFiles) {
    const targetFile = findTargetFile(sourceFile, target);
    const result = validateFilePair(sourceFile, targetFile, terms);
    results.push(result);

    // Log progress
    if (!json) {
      const status = result.passed ? '✓' : '✗';
      console.error(`${status} ${result.file}: ${result.preservedTerms}/${result.totalTerms} terms preserved`);

      if (!result.passed && result.missingTerms.length > 0) {
        console.error(`  Missing: ${result.missingTerms.join(', ')}`);
      }
    }
  }

  // Calculate summary
  const totalTermsChecked = results.reduce((sum, r) => sum + r.totalTerms, 0);
  const totalTermsPreserved = results.reduce((sum, r) => sum + r.preservedTerms, 0);

  const report: ValidationReport = {
    generated: new Date().toISOString(),
    strictMode: strict,
    results,
    summary: {
      totalFiles: results.length,
      passedFiles: results.filter((r) => r.passed).length,
      totalTermsChecked,
      totalTermsPreserved,
      overallPreservationRate: totalTermsChecked > 0
        ? totalTermsPreserved / totalTermsChecked
        : 1.0,
      allPassed: results.every((r) => r.passed),
    },
  };

  // Output
  if (json) {
    console.log(JSON.stringify(report, null, 2));
  } else {
    console.log('\n--- Validation Summary ---');
    console.log(`Total files: ${report.summary.totalFiles}`);
    console.log(`Passed files: ${report.summary.passedFiles}`);
    console.log(`Terms checked: ${report.summary.totalTermsChecked}`);
    console.log(`Terms preserved: ${report.summary.totalTermsPreserved}`);
    console.log(`Preservation rate: ${(report.summary.overallPreservationRate * 100).toFixed(1)}%`);
    console.log(`Status: ${report.summary.allPassed ? 'PASSED' : 'FAILED'}`);
  }

  // Exit with error if validation failed and strict mode
  if (strict && !report.summary.allPassed) {
    process.exit(1);
  }
}

main();
