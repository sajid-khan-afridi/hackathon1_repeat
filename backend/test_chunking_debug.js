#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Test with a sample file
const testFile = path.resolve(__dirname, '../docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx');
console.log(`Testing chunking with: ${testFile}`);
console.log('=' .repeat(60));

const content = fs.readFileSync(testFile, 'utf-8');
const { data: frontmatter, content: bodyContent } = matter(content);

console.log(`Title: ${frontmatter.title}`);
console.log(`Body word count: ${bodyContent.split(/\s+/).length}`);
console.log(`Body char count: ${bodyContent.length}`);

// Extract code blocks
const codeBlockRegex = /```[\s\S]*?```/g;
const codeBlocks = bodyContent.match(codeBlockRegex) || [];
console.log(`\nCode blocks found: ${codeBlocks.length}`);

// Show code block sizes
codeBlocks.forEach((block, idx) => {
  console.log(`  Block ${idx + 1}: ${block.length} chars, ${block.split(/\s+/).length} words`);
});

// Replace code blocks with placeholders
let textContent = bodyContent;
codeBlocks.forEach((block, i) => {
  textContent = textContent.replace(block, `__CODE_BLOCK_${i}__`);
});

console.log(`\nAfter removing code blocks:`);
console.log(`  Text content word count: ${textContent.split(/\s+/).length}`);
console.log(`  Text content char count: ${textContent.length}`);

// Split by paragraphs
const paragraphs = textContent.split(/\n\n+/);
console.log(`\nParagraphs: ${paragraphs.length}`);

// Show first 5 paragraphs
console.log(`\nFirst 5 paragraphs:`);
paragraphs.slice(0, 5).forEach((para, idx) => {
  console.log(`\n--- Para ${idx + 1} ---`);
  console.log(`Words: ${para.split(/\s+/).length}`);
  console.log(`Preview: ${para.substring(0, 100).replace(/\n/g, ' ')}...`);
});
