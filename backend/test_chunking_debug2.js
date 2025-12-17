#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Test with a sample file
const testFile = path.resolve(__dirname, '../docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx');
const content = fs.readFileSync(testFile, 'utf-8');
const { data: frontmatter, content: bodyContent } = matter(content);

// Extract code blocks
const codeBlockRegex = /```[\s\S]*?```/g;
const codeBlocks = bodyContent.match(codeBlockRegex) || [];

// Replace code blocks with placeholders
let textContent = bodyContent;
codeBlocks.forEach((block, i) => {
  textContent = textContent.replace(block, `__CODE_BLOCK_${i}__`);
});

// Show a sample of what the text looks like after replacement
console.log('Sample of text after code block replacement:');
console.log('='.repeat(60));
console.log(textContent.substring(0, 2000));
console.log('='.repeat(60));

// Check for double newlines
const doubleNewlineCount = (textContent.match(/\n\n/g) || []).length;
console.log(`\nDouble newlines found: ${doubleNewlineCount}`);

// Show what splitting by \n\n+ produces
const paragraphs = textContent.split(/\n\n+/);
console.log(`Paragraphs after split: ${paragraphs.length}`);
