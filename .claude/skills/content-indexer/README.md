# content-indexer

Indexes MDX chapter content into Qdrant vector database for semantic search and RAG chatbot.

## Overview

The content-indexer skill provides:
- Semantic chunking of MDX content
- Preservation of code blocks as separate chunks
- OpenAI embedding generation
- Qdrant vector storage with metadata
- Batch directory indexing
- Reindexing and deletion support

## Usage

### As a CLI Tool

```bash
# Index a single chapter
node index.js index_chapter --path docs/module-1/chapter-1.mdx

# Index all files in a directory
node index.js index_directory --path docs/

# Reindex all content (deletes existing and rebuilds)
node index.js reindex_all --path docs/

# Delete a chapter from the index
node index.js delete_chapter --path docs/module-1/chapter-1.mdx

# Custom chunking options
node index.js index_directory --path docs/ --chunk_size 300 --chunk_overlap 30
```

### As a Module

```javascript
import ContentIndexer from './index.js';

const indexer = new ContentIndexer({
  projectRoot: process.cwd(),
  collectionName: 'robotics_textbook',
  chunkSize: 500,
  chunkOverlap: 50,
  preserveCodeBlocks: true
});

// Index a single chapter
const result = await indexer.indexChapter({
  chapterPath: 'docs/module-1/chapter-1.mdx'
});

console.log('Chunks created:', result.total_chunks);
console.log('Tokens used:', result.total_tokens);

// Index entire docs directory
const batchResult = await indexer.indexDirectory({
  directoryPath: 'docs/'
});

console.log('Indexed chapters:', batchResult.indexed_chapters.length);
```

## Workflow

1. **Read MDX file(s)** and parse frontmatter
2. **Extract metadata** (chapter_id, module, title, tags)
3. **Split content** into semantic chunks
4. **Preserve code blocks** as separate chunks
5. **Generate embeddings** using OpenAI
6. **Upsert chunks** to Qdrant with metadata
7. **Return statistics**

## Chunking Strategy

The indexer uses a semantic chunking approach:

- **Text chunks**: Split by paragraphs, respecting word count limits
- **Code blocks**: Preserved as separate chunks (```...```)
- **Overlap**: Configurable word overlap between chunks
- **Size limits**: Default 500 words per chunk

### Critical: Preserving Paragraph Boundaries

⚠️ **IMPORTANT**: When replacing code blocks with placeholders, the algorithm MUST preserve double newlines (`\n\n`) to maintain paragraph boundaries. Without this:
- Entire chapters collapse into a single paragraph
- Chunking produces only 1 chunk per file instead of 20-30+ chunks
- Search quality degrades dramatically (34% → 56% confidence with proper chunking)

**Implementation:**
```javascript
// ❌ WRONG: Loses paragraph boundaries
textContent.replace(block, `__CODE_BLOCK_${i}__`)

// ✅ CORRECT: Preserves boundaries with double newlines
textContent.replace(block, `\n\n__CODE_BLOCK_${i}__\n\n`)
```

This fix increased chunking granularity by **24x** (15 → 363 chunks) and improved search confidence by **64%**.

## Vector Storage

Each chunk is stored in Qdrant with:

| Field | Type | Description |
|-------|------|-------------|
| chapter_id | keyword | Unique chapter identifier |
| chapter_title | text | Chapter title (⚠️ MUST be `chapter_title`, not `title`) |
| module | integer | Module number (for filtering) |
| tags | array | Content tags |
| chunk_index | integer | Position within chapter |
| chunk_type | keyword | 'text' or 'code' |
| text | text | Actual chunk content |
| word_count | integer | Word count of chunk |

### Critical: Backend Compatibility

⚠️ **IMPORTANT**: The payload MUST use `chapter_title` as the key name (not `title`). The backend's `vector_service.py` expects this exact field name:

```javascript
// ❌ WRONG: Backend won't find chapter titles
payload: { chapter_id, title, module, tags, ... }

// ✅ CORRECT: Backend expects 'chapter_title'
payload: { chapter_id, chapter_title: title, module, tags, ... }
```

Without this, all search results will show "Unknown Chapter" in the UI.

## Integration

This skill integrates with:
- **qdrant-vectorstore**: Stores generated embeddings and metadata
- **openai-agents-sdk**: Generates embeddings for content chunks
- **mdx-writer**: Indexes MDX files created by mdx-writer
- **rag-pipeline**: Provides searchable content for RAG queries

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| QDRANT_URL | Yes | Qdrant server URL |
| QDRANT_API_KEY | No | Qdrant API key (required for cloud) |
| OPENAI_API_KEY | Yes | OpenAI API key for embeddings |
| EMBEDDING_MODEL | No | OpenAI model (default: text-embedding-3-small) |

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| collectionName | robotics_textbook | Qdrant collection name |
| chunkSize | 500 | Maximum words per chunk |
| chunkOverlap | 50 | Overlapping words between chunks |
| preserveCodeBlocks | true | Keep code blocks as separate chunks |

## Output

```json
{
  "success": true,
  "indexed_chapters": [
    {
      "chapter_id": "docs-module-1-chapter-1",
      "title": "Introduction to ROS",
      "chunks_created": 12,
      "word_count": 3500,
      "tokens_used": 4200
    }
  ],
  "total_chunks": 12,
  "total_tokens": 4200,
  "errors": []
}
```

## Performance & Quality Metrics

### Expected Chunking Granularity

For optimal search quality, aim for these chunking metrics:

| File Size | Expected Chunks | Chunk Types |
|-----------|-----------------|-------------|
| 1000 words | 15-20 chunks | ~10 text + ~5-10 code |
| 2000 words | 20-25 chunks | ~11 text + ~10 code |
| 3000 words | 25-30 chunks | ~14 text + ~13 code |

**Red Flag:** If a 1800+ word chapter produces only 1-2 chunks, the chunking algorithm is broken (likely missing double newlines).

### Search Quality Impact

| Metric | Poor Chunking | Optimal Chunking | Impact |
|--------|---------------|------------------|--------|
| Chunks per file | 1-2 | 20-30 | 15-20x |
| Total chunks (15 files) | 15-30 | 300-400 | 10-20x |
| Confidence score | 30-40% | 50-60% | +50-70% |
| Relevance score | 30-40% | 50-60% | +50-70% |
| Chapter title accuracy | "Unknown" | Proper titles | Critical UX |

### Validation Checklist

After indexing, verify quality:

```bash
# 1. Check total chunks (should be 300-400 for ~15 MDX files)
python inspect_qdrant.py

# 2. Verify chapter_title field exists (not "title")
# Should show: chapter_title: "Chapter 1: ..."
# NOT: chapter_title: NOT FOUND

# 3. Test search quality
curl -X POST https://your-backend.com/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?"}'

# Expected: confidence ≥ 50%, relevance ≥ 50%, proper chapter titles
```

## Troubleshooting

### Issue: Only 1 chunk per file

**Cause:** Code block placeholders not preserving double newlines
**Fix:** Ensure replacement uses `\n\n__CODE_BLOCK_${i}__\n\n`
**File:** `index.js` line 285

### Issue: "Unknown Chapter" in search results

**Cause:** Payload uses `title` instead of `chapter_title`
**Fix:** Change payload to `chapter_title: title`
**File:** `index.js` line 244

### Issue: Low confidence/relevance scores

**Cause:** Insufficient chunking granularity
**Fix:** Verify paragraph boundaries preserved, check chunk count
**Expected:** 20-30 chunks per 2000-word file
