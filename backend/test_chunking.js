#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Simple chunking function (copied from content-indexer)
function splitIntoChunks(content, chunkSize = 500, chunkOverlap = 50) {
  const chunks = [];

  // Extract code blocks
  const codeBlockRegex = /```[\s\S]*?```/g;
  const codeBlocks = content.match(codeBlockRegex) || [];
  let textContent = content;

  // Replace code blocks with placeholders
  codeBlocks.forEach((block, i) => {
    textContent = textContent.replace(block, `__CODE_BLOCK_${i}__`);
  });

  // Split by paragraphs
  const paragraphs = textContent.split(/\n\n+/);
  let currentChunk = '';
  let currentWordCount = 0;

  for (const para of paragraphs) {
    const paraWords = para.split(/\s+/).length;

    // Check for code block placeholder
    const codeMatch = para.match(/__CODE_BLOCK_(\d+)__/);
    if (codeMatch) {
      if (currentChunk.trim()) {
        chunks.push({ text: currentChunk.trim(), type: 'text' });
      }
      currentChunk = '';
      currentWordCount = 0;

      const blockIndex = parseInt(codeMatch[1]);
      chunks.push({ text: codeBlocks[blockIndex], type: 'code' });
      continue;
    }

    // Add to current chunk if it fits
    if (currentWordCount + paraWords <= chunkSize) {
      currentChunk += (currentChunk ? '\n\n' : '') + para;
      currentWordCount += paraWords;
    } else {
      if (currentChunk.trim()) {
        chunks.push({ text: currentChunk.trim(), type: 'text' });
      }

      // Handle overlap
      if (chunkOverlap > 0) {
        const words = currentChunk.split(/\s+/);
        const overlapWords = words.slice(-chunkOverlap).join(' ');
        currentChunk = overlapWords + '\n\n' + para;
        currentWordCount = chunkOverlap + paraWords;
      } else {
        currentChunk = para;
        currentWordCount = paraWords;
      }
    }
  }

  if (currentChunk.trim()) {
    chunks.push({ text: currentChunk.trim(), type: 'text' });
  }

  return chunks;
}

// Test with a sample file
const testFile = path.resolve(__dirname, '../docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx');
console.log(`Testing chunking with: ${testFile}`);
console.log('=' .repeat(60));

const content = fs.readFileSync(testFile, 'utf-8');
const { data: frontmatter, content: bodyContent } = matter(content);

console.log(`Title: ${frontmatter.title}`);
console.log(`Body word count: ${bodyContent.split(/\s+/).length}`);

const chunks = splitIntoChunks(bodyContent, 500, 50);
console.log(`\nChunks created: ${chunks.length}`);

chunks.forEach((chunk, idx) => {
  const wordCount = chunk.text.split(/\s+/).length;
  console.log(`\n--- Chunk ${idx + 1} (${chunk.type}) ---`);
  console.log(`Word count: ${wordCount}`);
  console.log(`Preview: ${chunk.text.substring(0, 100)}...`);
});
