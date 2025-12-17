#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Updated chunking function with fix
function splitIntoChunks(content, chunkSize = 500, chunkOverlap = 50) {
  const chunks = [];

  // Extract code blocks
  const codeBlockRegex = /```[\s\S]*?```/g;
  const codeBlocks = content.match(codeBlockRegex) || [];
  let textContent = content;

  // Replace code blocks with placeholders (preserve paragraph boundaries)
  codeBlocks.forEach((block, i) => {
    textContent = textContent.replace(block, `\n\n__CODE_BLOCK_${i}__\n\n`);
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

// Test with sample files
const testFiles = [
  '../docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx',
  '../docs/module-2-isaac-sim/chapter-1-introduction.mdx',
  '../docs/module-3-applications/chapter-1-kinematics.mdx'
];

let totalChunks = 0;

testFiles.forEach(relativePath => {
  const testFile = path.resolve(__dirname, relativePath);
  const content = fs.readFileSync(testFile, 'utf-8');
  const { data: frontmatter, content: bodyContent } = matter(content);

  const chunks = splitIntoChunks(bodyContent, 500, 50);
  totalChunks += chunks.length;

  console.log(`\n${'='.repeat(60)}`);
  console.log(`File: ${path.basename(testFile)}`);
  console.log(`Title: ${frontmatter.title}`);
  console.log(`Body words: ${bodyContent.split(/\s+/).length}`);
  console.log(`Chunks created: ${chunks.length}`);

  const textChunks = chunks.filter(c => c.type === 'text').length;
  const codeChunks = chunks.filter(c => c.type === 'code').length;
  console.log(`  - Text chunks: ${textChunks}`);
  console.log(`  - Code chunks: ${codeChunks}`);
});

console.log(`\n${'='.repeat(60)}`);
console.log(`TOTAL CHUNKS: ${totalChunks}`);
console.log(`${'='.repeat(60)}`);
