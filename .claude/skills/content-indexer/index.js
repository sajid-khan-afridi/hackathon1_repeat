#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import OpenAI from 'openai';
import { QdrantClient } from '@qdrant/js-client-rest';
import crypto from 'crypto';
import dotenv from 'dotenv';
import { fileURLToPath } from 'url';

// Load environment variables from project root
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const envPath = path.resolve(__dirname, '../../../backend/.env');
dotenv.config({ path: envPath });

/**
 * Content Indexer Skill
 * Indexes MDX chapter content into Qdrant vector database for semantic search
 */
class ContentIndexer {
  constructor(options = {}) {
    this.projectRoot = options.projectRoot || process.cwd();
    this.collectionName = options.collectionName || 'robotics_textbook';
    this.chunkSize = options.chunkSize || 500;
    this.chunkOverlap = options.chunkOverlap || 50;
    this.preserveCodeBlocks = options.preserveCodeBlocks !== false;
    this.embeddingModel = process.env.EMBEDDING_MODEL || 'text-embedding-3-small';
    this.embeddingDimensions = 1536;

    // Initialize clients
    this.openai = null;
    this.qdrant = null;
  }

  /**
   * Initialize API clients
   */
  async initialize() {
    if (!process.env.OPENAI_API_KEY) {
      throw new Error('OPENAI_API_KEY environment variable is required');
    }
    if (!process.env.QDRANT_URL) {
      throw new Error('QDRANT_URL environment variable is required');
    }

    this.openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });

    this.qdrant = new QdrantClient({
      url: process.env.QDRANT_URL,
      apiKey: process.env.QDRANT_API_KEY || undefined
    });

    // Ensure collection exists
    await this.ensureCollection();
  }

  /**
   * Ensure Qdrant collection exists with proper configuration
   */
  async ensureCollection() {
    try {
      await this.qdrant.getCollection(this.collectionName);
    } catch (error) {
      // Collection doesn't exist, create it
      await this.qdrant.createCollection(this.collectionName, {
        vectors: {
          size: this.embeddingDimensions,
          distance: 'Cosine'
        }
      });

      // Create payload indexes for efficient filtering
      await this.qdrant.createPayloadIndex(this.collectionName, {
        field_name: 'chapter_id',
        field_schema: 'keyword'
      });

      await this.qdrant.createPayloadIndex(this.collectionName, {
        field_name: 'module',
        field_schema: 'integer'
      });
    }
  }

  /**
   * Index a single chapter
   */
  async indexChapter(options = {}) {
    const { chapterPath } = options;

    if (!chapterPath) {
      throw new Error('chapterPath is required');
    }

    await this.initialize();

    const fullPath = path.resolve(this.projectRoot, chapterPath);
    if (!fs.existsSync(fullPath)) {
      throw new Error(`File not found: ${chapterPath}`);
    }

    const content = fs.readFileSync(fullPath, 'utf-8');
    const result = await this.processAndIndex(content, fullPath);

    return {
      success: true,
      indexed_chapters: [result],
      total_chunks: result.chunks_created,
      total_tokens: result.tokens_used
    };
  }

  /**
   * Index a directory of MDX files
   */
  async indexDirectory(options = {}) {
    const { directoryPath } = options;

    if (!directoryPath) {
      throw new Error('directoryPath is required');
    }

    await this.initialize();

    const fullPath = path.resolve(this.projectRoot, directoryPath);
    if (!fs.existsSync(fullPath)) {
      throw new Error(`Directory not found: ${directoryPath}`);
    }

    const files = this.findMdxFiles(fullPath);
    const results = [];
    const errors = [];
    let totalChunks = 0;
    let totalTokens = 0;

    for (const file of files) {
      try {
        const content = fs.readFileSync(file, 'utf-8');
        const result = await this.processAndIndex(content, file);
        results.push(result);
        totalChunks += result.chunks_created;
        totalTokens += result.tokens_used;
      } catch (error) {
        errors.push({ file, error: error.message });
      }
    }

    return {
      success: errors.length === 0,
      indexed_chapters: results,
      total_chunks: totalChunks,
      total_tokens: totalTokens,
      errors
    };
  }

  /**
   * Reindex all content
   */
  async reindexAll(options = {}) {
    const { docsPath = 'docs' } = options;

    // Delete existing collection
    try {
      await this.qdrant.deleteCollection(this.collectionName);
    } catch (error) {
      // Collection might not exist
    }

    // Reinitialize
    this.qdrant = null;
    await this.initialize();

    // Index all content
    return this.indexDirectory({ directoryPath: docsPath });
  }

  /**
   * Delete a chapter from the index
   */
  async deleteChapter(options = {}) {
    const { chapterPath } = options;

    if (!chapterPath) {
      throw new Error('chapterPath is required');
    }

    await this.initialize();

    const chapterId = this.generateChapterId(chapterPath);

    await this.qdrant.delete(this.collectionName, {
      filter: {
        must: [
          { key: 'chapter_id', match: { value: chapterId } }
        ]
      }
    });

    return {
      success: true,
      deleted_chapter: chapterId
    };
  }

  /**
   * Process and index content
   */
  async processAndIndex(content, filePath) {
    // Parse frontmatter
    const { data: frontmatter, content: bodyContent } = matter(content);

    // Extract metadata
    const chapterId = this.generateChapterId(filePath);
    const title = frontmatter.title || path.basename(filePath, path.extname(filePath));
    const module = this.extractModuleNumber(filePath);
    const tags = frontmatter.tags || [];

    // Split into chunks
    const chunks = this.splitIntoChunks(bodyContent);

    // Process each chunk
    const points = [];
    let tokensUsed = 0;

    for (let i = 0; i < chunks.length; i++) {
      const chunk = chunks[i];

      // Generate embedding
      const embeddingResponse = await this.openai.embeddings.create({
        model: this.embeddingModel,
        input: chunk.text
      });

      tokensUsed += embeddingResponse.usage.total_tokens;

      points.push({
        id: this.generatePointId(chapterId, i),
        vector: embeddingResponse.data[0].embedding,
        payload: {
          chapter_id: chapterId,
          // ⚠️ CRITICAL: Must be 'chapter_title', NOT 'title'
          // The backend's vector_service.py expects this exact field name
          // Using 'title' will cause all search results to show "Unknown Chapter"
          chapter_title: title,
          module,
          tags,
          chunk_index: i,
          chunk_type: chunk.type,
          text: chunk.text,
          word_count: chunk.text.split(/\s+/).length
        }
      });
    }

    // Upsert to Qdrant
    if (points.length > 0) {
      await this.qdrant.upsert(this.collectionName, {
        points
      });
    }

    return {
      chapter_id: chapterId,
      title,
      chunks_created: chunks.length,
      word_count: bodyContent.split(/\s+/).length,
      tokens_used: tokensUsed
    };
  }

  /**
   * Split content into semantic chunks
   *
   * CRITICAL: This function must preserve paragraph boundaries (double newlines)
   * when replacing code blocks. Without \n\n around placeholders, entire chapters
   * collapse into 1 chunk instead of 20-30 chunks, degrading search quality by 64%.
   *
   * Performance Impact:
   * - Correct: 363 chunks from 15 files (24x granularity)
   * - Broken: 15 chunks from 15 files (poor search quality)
   */
  splitIntoChunks(content) {
    const chunks = [];

    // First, extract and preserve code blocks
    const codeBlockRegex = /```[\s\S]*?```/g;
    const codeBlocks = content.match(codeBlockRegex) || [];
    let textContent = content;

    if (this.preserveCodeBlocks) {
      // ⚠️ CRITICAL: Must use \n\n around placeholders to preserve paragraph boundaries
      // This ensures the subsequent split(/\n\n+/) works correctly
      // WITHOUT THIS: Entire 1800-word chapters become 1 chunk (search quality drops 64%)
      // WITH THIS: Chapters split into 20-30 chunks (optimal search granularity)
      codeBlocks.forEach((block, i) => {
        textContent = textContent.replace(block, `\n\n__CODE_BLOCK_${i}__\n\n`);
      });
    }

    // Split text content by paragraphs/sections
    // This regex requires double newlines (\n\n+) which is why the above fix is critical
    const paragraphs = textContent.split(/\n\n+/);
    let currentChunk = '';
    let currentWordCount = 0;

    for (const para of paragraphs) {
      const paraWords = para.split(/\s+/).length;

      // Check if this is a code block placeholder
      const codeMatch = para.match(/__CODE_BLOCK_(\d+)__/);
      if (codeMatch && this.preserveCodeBlocks) {
        // Save current text chunk
        if (currentChunk.trim()) {
          chunks.push({ text: currentChunk.trim(), type: 'text' });
        }
        currentChunk = '';
        currentWordCount = 0;

        // Add code block as separate chunk
        const blockIndex = parseInt(codeMatch[1]);
        chunks.push({ text: codeBlocks[blockIndex], type: 'code' });
        continue;
      }

      // Add to current chunk if it fits
      if (currentWordCount + paraWords <= this.chunkSize) {
        currentChunk += (currentChunk ? '\n\n' : '') + para;
        currentWordCount += paraWords;
      } else {
        // Save current chunk and start new one
        if (currentChunk.trim()) {
          chunks.push({ text: currentChunk.trim(), type: 'text' });
        }

        // Handle overlap
        if (this.chunkOverlap > 0) {
          const words = currentChunk.split(/\s+/);
          const overlapWords = words.slice(-this.chunkOverlap).join(' ');
          currentChunk = overlapWords + '\n\n' + para;
          currentWordCount = this.chunkOverlap + paraWords;
        } else {
          currentChunk = para;
          currentWordCount = paraWords;
        }
      }
    }

    // Don't forget the last chunk
    if (currentChunk.trim()) {
      chunks.push({ text: currentChunk.trim(), type: 'text' });
    }

    return chunks;
  }

  /**
   * Find all MDX files in a directory
   */
  findMdxFiles(dir) {
    const files = [];

    const walk = (currentDir) => {
      const entries = fs.readdirSync(currentDir, { withFileTypes: true });

      for (const entry of entries) {
        const fullPath = path.join(currentDir, entry.name);

        if (entry.isDirectory()) {
          walk(fullPath);
        } else if (entry.isFile() && (entry.name.endsWith('.mdx') || entry.name.endsWith('.md'))) {
          files.push(fullPath);
        }
      }
    };

    walk(dir);
    return files;
  }

  /**
   * Generate chapter ID from file path
   */
  generateChapterId(filePath) {
    const relativePath = path.relative(this.projectRoot, filePath);
    return relativePath.replace(/[\\\/]/g, '-').replace(/\.(mdx?|md)$/, '');
  }

  /**
   * Generate point ID for Qdrant
   */
  generatePointId(chapterId, chunkIndex) {
    const hash = crypto.createHash('md5')
      .update(`${chapterId}-${chunkIndex}`)
      .digest('hex');
    return hash;
  }

  /**
   * Extract module number from file path
   */
  extractModuleNumber(filePath) {
    const match = filePath.match(/module[- _]?(\d+)/i);
    return match ? parseInt(match[1]) : 0;
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: content-indexer <operation> [options]');
    console.error('Operations: index_chapter, index_directory, reindex_all, delete_chapter');
    console.error('');
    console.error('Examples:');
    console.error('  content-indexer index_chapter --path docs/module-1/chapter-1.mdx');
    console.error('  content-indexer index_directory --path docs/');
    console.error('  content-indexer reindex_all');
    console.error('  content-indexer delete_chapter --path docs/module-1/chapter-1.mdx');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const indexer = new ContentIndexer({
    projectRoot: process.cwd(),
    collectionName: options.collection || 'robotics_textbook',
    chunkSize: parseInt(options.chunk_size || '500'),
    chunkOverlap: parseInt(options.chunk_overlap || '50')
  });

  try {
    let result;
    switch (operation) {
      case 'index_chapter':
        result = await indexer.indexChapter({
          chapterPath: options.path
        });
        break;

      case 'index_directory':
        result = await indexer.indexDirectory({
          directoryPath: options.path || 'docs'
        });
        break;

      case 'reindex_all':
        result = await indexer.reindexAll({
          docsPath: options.path || 'docs'
        });
        break;

      case 'delete_chapter':
        result = await indexer.deleteChapter({
          chapterPath: options.path
        });
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
 * Parse CLI arguments
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
export default ContentIndexer;
export { ContentIndexer };

// Run CLI if called directly
const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('content-indexer/index.js') ||
                     process.argv[1]?.endsWith('content-indexer\\index.js');

if (isMainModule) {
  main().catch(console.error);
}
