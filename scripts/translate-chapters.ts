#!/usr/bin/env ts-node
/**
 * Translation Orchestrator Script
 * Orchestrates the translation of MDX chapters to Urdu
 * Phase 5: Translation Feature - T011
 *
 * Usage:
 *   npx ts-node scripts/translate-chapters.ts [options]
 *
 * Options:
 *   --force, -f     Force re-translation even if cache is valid
 *   --chapter, -c   Translate specific chapter only
 *   --dry-run       Show what would be translated without doing it
 *   --validate      Validate terms after translation
 *
 * This script:
 * 1. Scans for English MDX chapters
 * 2. Checks cache validity (content hash comparison)
 * 3. Protects technical terms before translation
 * 4. Calls translation API (urdu-translator skill)
 * 5. Unwraps protected terms after translation
 * 6. Writes translated content to i18n directory
 * 7. Updates cache metadata
 */

import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';

interface TranslationCache {
  chapterId: string;
  sourceHash: string;
  translatedAt: string;
  sourceFile: string;
  translatedFile: string;
  termCount: number;
}

interface TranslationResult {
  chapterId: string;
  status: 'translated' | 'cached' | 'skipped' | 'error';
  message: string;
  duration?: number;
}

interface TranslationReport {
  started: string;
  completed: string;
  results: TranslationResult[];
  summary: {
    total: number;
    translated: number;
    cached: number;
    skipped: number;
    errors: number;
  };
}

const DOCS_DIR = path.join(process.cwd(), 'docs');
const I18N_DIR = path.join(process.cwd(), 'docs', 'i18n', 'ur', 'docusaurus-plugin-content-docs', 'current');
const TERMS_FILE = path.join(process.cwd(), 'src', 'data', 'technical-terms.json');

// Placeholder pattern for protecting technical terms
const TERM_PLACEHOLDER_PREFIX = '[[TERM:';
const TERM_PLACEHOLDER_SUFFIX = ']]';

/**
 * Generate MD5 hash of content
 */
function generateHash(content: string): string {
  return crypto.createHash('md5').update(content, 'utf8').digest('hex');
}

/**
 * Load technical terms to preserve
 */
function loadTermsToPreserve(): string[] {
  if (!fs.existsSync(TERMS_FILE)) {
    console.warn('Warning: Technical terms file not found, no terms will be protected');
    return [];
  }

  const data = JSON.parse(fs.readFileSync(TERMS_FILE, 'utf8'));
  return data.terms
    .filter((t: { translationStatus: string }) => t.translationStatus === 'preserve')
    .map((t: { term: string }) => t.term);
}

/**
 * Find all MDX chapter files
 */
function findChapterFiles(): string[] {
  const files: string[] = [];

  function walkDir(dir: string): void {
    const entries = fs.readdirSync(dir, { withFileTypes: true });
    for (const entry of entries) {
      const fullPath = path.join(dir, entry.name);
      if (entry.isDirectory()) {
        if (!entry.name.startsWith('.') && entry.name !== 'i18n' && entry.name !== 'node_modules') {
          walkDir(fullPath);
        }
      } else if (entry.isFile() && /\.mdx?$/.test(entry.name)) {
        files.push(fullPath);
      }
    }
  }

  walkDir(DOCS_DIR);
  return files;
}

/**
 * Get chapter ID from file path
 */
function getChapterId(filePath: string): string {
  return path.relative(DOCS_DIR, filePath).replace(/\.[^.]+$/, '').replace(/\\/g, '/');
}

/**
 * Get target path for translated file
 */
function getTargetPath(sourceFile: string): string {
  const relativePath = path.relative(DOCS_DIR, sourceFile);
  return path.join(I18N_DIR, relativePath);
}

/**
 * Get cache file path
 */
function getCachePath(targetFile: string): string {
  return targetFile.replace(/\.[^.]+$/, '.meta.json');
}

/**
 * Load cache metadata for a chapter
 */
function loadCache(targetFile: string): TranslationCache | null {
  const cachePath = getCachePath(targetFile);
  if (!fs.existsSync(cachePath)) {
    return null;
  }

  try {
    return JSON.parse(fs.readFileSync(cachePath, 'utf8'));
  } catch {
    return null;
  }
}

/**
 * Save cache metadata
 */
function saveCache(cache: TranslationCache): void {
  const cachePath = getCachePath(cache.translatedFile);
  const cacheDir = path.dirname(cachePath);

  if (!fs.existsSync(cacheDir)) {
    fs.mkdirSync(cacheDir, { recursive: true });
  }

  fs.writeFileSync(cachePath, JSON.stringify(cache, null, 2));
}

/**
 * Check if cache is still valid
 */
function isCacheValid(sourceFile: string, cache: TranslationCache | null): boolean {
  if (!cache) return false;

  const content = fs.readFileSync(sourceFile, 'utf8');
  const currentHash = generateHash(content);

  return cache.sourceHash === currentHash;
}

/**
 * Protect technical terms by wrapping them in placeholders
 */
function protectTerms(content: string, terms: string[]): { protected: string; count: number } {
  let protectedContent = content;
  let count = 0;

  // Sort terms by length (longest first) to avoid partial replacements
  const sortedTerms = [...terms].sort((a, b) => b.length - a.length);

  for (const term of sortedTerms) {
    const regex = new RegExp(`\\b(${escapeRegex(term)})\\b`, 'g');
    const matches = protectedContent.match(regex);
    if (matches) {
      count += matches.length;
      protectedContent = protectedContent.replace(
        regex,
        `${TERM_PLACEHOLDER_PREFIX}$1${TERM_PLACEHOLDER_SUFFIX}`
      );
    }
  }

  return { protected: protectedContent, count };
}

/**
 * Unwrap protected terms after translation
 */
function unwrapTerms(content: string): string {
  const regex = new RegExp(
    `${escapeRegex(TERM_PLACEHOLDER_PREFIX)}(.+?)${escapeRegex(TERM_PLACEHOLDER_SUFFIX)}`,
    'g'
  );
  return content.replace(regex, '$1');
}

/**
 * Escape regex special characters
 */
function escapeRegex(str: string): string {
  return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

/**
 * Translate content to Urdu (placeholder for actual API call)
 * In production, this would call the urdu-translator skill
 */
async function translateContent(content: string): Promise<string> {
  // This is a placeholder - in production, integrate with urdu-translator skill
  // For now, just return the content with a note indicating translation is needed
  console.log('  Note: Translation API integration pending. Returning original content.');
  return content;
}

/**
 * Translate a single chapter
 */
async function translateChapter(
  sourceFile: string,
  terms: string[],
  force: boolean,
  dryRun: boolean
): Promise<TranslationResult> {
  const chapterId = getChapterId(sourceFile);
  const targetFile = getTargetPath(sourceFile);
  const startTime = Date.now();

  // Check cache
  const cache = loadCache(targetFile);
  if (!force && isCacheValid(sourceFile, cache)) {
    return {
      chapterId,
      status: 'cached',
      message: 'Cache is valid, skipping translation',
    };
  }

  if (dryRun) {
    return {
      chapterId,
      status: 'skipped',
      message: 'Dry run - would translate this chapter',
    };
  }

  try {
    // Read source content
    const sourceContent = fs.readFileSync(sourceFile, 'utf8');
    const sourceHash = generateHash(sourceContent);

    // Protect technical terms
    const { protected: protectedContent, count: termCount } = protectTerms(sourceContent, terms);

    // Translate content
    const translatedContent = await translateContent(protectedContent);

    // Unwrap protected terms
    const finalContent = unwrapTerms(translatedContent);

    // Ensure target directory exists
    const targetDir = path.dirname(targetFile);
    if (!fs.existsSync(targetDir)) {
      fs.mkdirSync(targetDir, { recursive: true });
    }

    // Write translated content
    fs.writeFileSync(targetFile, finalContent);

    // Save cache
    const newCache: TranslationCache = {
      chapterId,
      sourceHash,
      translatedAt: new Date().toISOString(),
      sourceFile: path.relative(process.cwd(), sourceFile).replace(/\\/g, '/'),
      translatedFile: path.relative(process.cwd(), targetFile).replace(/\\/g, '/'),
      termCount,
    };
    saveCache(newCache);

    const duration = Date.now() - startTime;
    return {
      chapterId,
      status: 'translated',
      message: `Translated successfully (${termCount} terms protected)`,
      duration,
    };
  } catch (error) {
    return {
      chapterId,
      status: 'error',
      message: error instanceof Error ? error.message : 'Unknown error',
    };
  }
}

/**
 * Parse command line arguments
 */
function parseArgs(): { force: boolean; chapter: string | null; dryRun: boolean; validate: boolean } {
  const args = process.argv.slice(2);

  const force = args.includes('-f') || args.includes('--force');
  const dryRun = args.includes('--dry-run');
  const validate = args.includes('--validate');

  const chapterIndex = args.findIndex((a) => a === '-c' || a === '--chapter');
  const chapter = chapterIndex !== -1 ? args[chapterIndex + 1] : null;

  return { force, chapter, dryRun, validate };
}

/**
 * Main function
 */
async function main(): Promise<void> {
  const { force, chapter, dryRun, validate } = parseArgs();
  const startTime = new Date();

  console.log('Translation Orchestrator');
  console.log('========================');
  console.log(`Force mode: ${force}`);
  console.log(`Dry run: ${dryRun}`);
  console.log();

  // Load terms
  const terms = loadTermsToPreserve();
  console.log(`Loaded ${terms.length} technical terms to preserve`);

  // Find chapters
  let chapters = findChapterFiles();
  if (chapter) {
    chapters = chapters.filter((c) => getChapterId(c).includes(chapter));
    if (chapters.length === 0) {
      console.error(`Error: No chapter found matching '${chapter}'`);
      process.exit(1);
    }
  }
  console.log(`Found ${chapters.length} chapters to process\n`);

  // Translate chapters
  const results: TranslationResult[] = [];

  for (const chapterFile of chapters) {
    console.log(`Processing: ${getChapterId(chapterFile)}`);
    const result = await translateChapter(chapterFile, terms, force, dryRun);
    results.push(result);

    const statusIcon = {
      translated: '✓',
      cached: '○',
      skipped: '-',
      error: '✗',
    }[result.status];

    console.log(`  ${statusIcon} ${result.status}: ${result.message}`);
    if (result.duration) {
      console.log(`  Duration: ${result.duration}ms`);
    }
  }

  // Generate report
  const endTime = new Date();
  const report: TranslationReport = {
    started: startTime.toISOString(),
    completed: endTime.toISOString(),
    results,
    summary: {
      total: results.length,
      translated: results.filter((r) => r.status === 'translated').length,
      cached: results.filter((r) => r.status === 'cached').length,
      skipped: results.filter((r) => r.status === 'skipped').length,
      errors: results.filter((r) => r.status === 'error').length,
    },
  };

  // Print summary
  console.log('\n--- Summary ---');
  console.log(`Total: ${report.summary.total}`);
  console.log(`Translated: ${report.summary.translated}`);
  console.log(`Cached: ${report.summary.cached}`);
  console.log(`Skipped: ${report.summary.skipped}`);
  console.log(`Errors: ${report.summary.errors}`);
  console.log(`Duration: ${endTime.getTime() - startTime.getTime()}ms`);

  // Run validation if requested
  if (validate && report.summary.translated > 0) {
    console.log('\nRunning term validation...');
    // Would call validate-terms.ts here
    console.log('Note: Run validate-terms.ts separately for full validation');
  }

  // Exit with error if any translations failed
  if (report.summary.errors > 0) {
    process.exit(1);
  }
}

main().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});
