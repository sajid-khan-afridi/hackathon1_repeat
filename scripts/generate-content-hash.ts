#!/usr/bin/env ts-node
/**
 * Content Hash Generation Script
 * Generates MD5 hashes for MDX chapter content to enable cache invalidation
 * Phase 5: Translation Feature - T009
 *
 * Usage:
 *   npx ts-node scripts/generate-content-hash.ts [options]
 *
 * Options:
 *   --output, -o    Output file path (default: stdout)
 *   --json          Output as JSON (default: text)
 *   --chapter, -c   Specific chapter to hash (optional)
 */

import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';

interface ContentHash {
  chapterId: string;
  filePath: string;
  hash: string;
  lastModified: string;
  contentLength: number;
}

interface HashResult {
  generated: string;
  totalChapters: number;
  hashes: ContentHash[];
}

const DOCS_DIR = path.join(process.cwd(), 'docs');
const CHAPTER_PATTERN = /\.mdx?$/;

/**
 * Generate MD5 hash of file content
 */
function generateHash(content: string): string {
  return crypto.createHash('md5').update(content, 'utf8').digest('hex');
}

/**
 * Extract chapter ID from file path
 */
function getChapterId(filePath: string): string {
  const relativePath = path.relative(DOCS_DIR, filePath);
  return relativePath.replace(/\.[^.]+$/, '').replace(/\\/g, '/');
}

/**
 * Find all MDX chapter files
 */
function findChapterFiles(dir: string, files: string[] = []): string[] {
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);

    if (entry.isDirectory()) {
      // Skip i18n, node_modules, and hidden directories
      if (!entry.name.startsWith('.') && entry.name !== 'i18n' && entry.name !== 'node_modules') {
        findChapterFiles(fullPath, files);
      }
    } else if (entry.isFile() && CHAPTER_PATTERN.test(entry.name)) {
      files.push(fullPath);
    }
  }

  return files;
}

/**
 * Generate hash for a single file
 */
function hashFile(filePath: string): ContentHash {
  const content = fs.readFileSync(filePath, 'utf8');
  const stats = fs.statSync(filePath);

  return {
    chapterId: getChapterId(filePath),
    filePath: path.relative(process.cwd(), filePath).replace(/\\/g, '/'),
    hash: generateHash(content),
    lastModified: stats.mtime.toISOString(),
    contentLength: content.length,
  };
}

/**
 * Main function
 */
function main(): void {
  const args = process.argv.slice(2);
  const outputJson = args.includes('--json');
  const outputIndex = args.indexOf('-o');
  const outputFile = outputIndex !== -1 ? args[outputIndex + 1] : null;
  const chapterIndex = args.indexOf('-c');
  const specificChapter = chapterIndex !== -1 ? args[chapterIndex + 1] : null;

  // Check if docs directory exists
  if (!fs.existsSync(DOCS_DIR)) {
    console.error(`Error: Docs directory not found at ${DOCS_DIR}`);
    process.exit(1);
  }

  // Find chapter files
  let files = findChapterFiles(DOCS_DIR);

  // Filter to specific chapter if provided
  if (specificChapter) {
    files = files.filter((f) => getChapterId(f).includes(specificChapter));
    if (files.length === 0) {
      console.error(`Error: No chapter found matching '${specificChapter}'`);
      process.exit(1);
    }
  }

  // Generate hashes
  const hashes = files.map(hashFile);

  const result: HashResult = {
    generated: new Date().toISOString(),
    totalChapters: hashes.length,
    hashes,
  };

  // Output
  const output = outputJson
    ? JSON.stringify(result, null, 2)
    : hashes.map((h) => `${h.hash}  ${h.filePath}`).join('\n');

  if (outputFile) {
    fs.writeFileSync(outputFile, output + '\n');
    console.log(`Hashes written to ${outputFile}`);
  } else {
    console.log(output);
  }

  // Summary
  if (!outputFile) {
    console.error(`\n--- Summary ---`);
    console.error(`Total chapters: ${hashes.length}`);
    console.error(`Generated: ${result.generated}`);
  }
}

main();
