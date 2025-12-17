# Content Indexer - Best Practices & Lessons Learned

## Critical Fixes for Optimal Search Quality

This document captures the critical lessons learned from debugging and optimizing the content-indexer skill for maximum RAG chatbot performance.

---

## 🔴 Critical Fix #1: Preserve Paragraph Boundaries

### The Problem
When replacing code blocks with placeholders, if you don't preserve double newlines (`\n\n`), the entire chapter collapses into a single paragraph. This causes catastrophic chunking failure.

### Impact
- **Before:** 15 chunks from 15 files (1 chunk per file)
- **After:** 363 chunks from 15 files (24x improvement)
- **Search Quality:** +64% confidence score, +62% relevance score

### The Fix

**Location:** `index.js` line 296

```javascript
// ❌ WRONG: Loses paragraph boundaries
codeBlocks.forEach((block, i) => {
  textContent = textContent.replace(block, `__CODE_BLOCK_${i}__`);
});

// ✅ CORRECT: Preserves paragraph boundaries
codeBlocks.forEach((block, i) => {
  textContent = textContent.replace(block, `\n\n__CODE_BLOCK_${i}__\n\n`);
});
```

### Why It Matters

The paragraph split relies on double newlines:
```javascript
const paragraphs = textContent.split(/\n\n+/);
```

Without double newlines around placeholders:
- Before: `...text\n\n```code```\n\n...text` → Many paragraphs ✅
- After: `...text__CODE_BLOCK_0__...text` → ONE paragraph ❌

### Validation

```bash
# Test chunking on a sample file (should show 20+ chunks for 1800 words)
node test_chunking_fixed.js

# Expected output:
# File: chapter-1-nodes-lifecycle.mdx
# Body words: 1812
# Chunks created: 21  ← GOOD!
#   - Text chunks: 11
#   - Code chunks: 10
```

---

## 🔴 Critical Fix #2: Backend Field Name Compatibility

### The Problem
The content-indexer was storing chapter titles as `title` in Qdrant, but the backend's `vector_service.py` expects `chapter_title`. This caused all search results to show "Unknown Chapter".

### Impact
- **Before:** All sources show "Unknown Chapter"
- **After:** Proper chapter titles ("Chapter 1: Introduction to Isaac Sim")
- **User Experience:** Critical for credibility and navigation

### The Fix

**Location:** `index.js` line 247

```javascript
// ❌ WRONG: Backend won't find it
payload: {
  chapter_id: chapterId,
  title: title,  // Backend looks for 'chapter_title'
  module,
  tags,
  // ...
}

// ✅ CORRECT: Match backend expectation
payload: {
  chapter_id: chapterId,
  chapter_title: title,  // Backend finds this
  module,
  tags,
  // ...
}
```

### Backend Code Reference

**File:** `backend/app/services/vector_service.py` line 86

```python
# Backend expects this exact field name
chapter_title = payload.get("chapter_title", "Unknown Chapter")
```

### Validation

```bash
# Inspect what's stored in Qdrant
python backend/inspect_qdrant.py

# Expected output:
# chapter_title: Chapter 1: Introduction to Isaac Sim  ← GOOD!
# NOT: chapter_title: NOT FOUND  ← BAD!
```

---

## 📊 Performance Metrics & Benchmarks

### Expected Chunking Granularity

| File Size (words) | Expected Chunks | Text Chunks | Code Chunks |
|-------------------|-----------------|-------------|-------------|
| 1,000 | 15-20 | ~10 | ~5-10 |
| 1,800 | 20-25 | ~11 | ~10 |
| 2,500 | 25-30 | ~14 | ~12 |
| 3,000 | 27-33 | ~14 | ~13 |

**Red Flag:** If a 1800-word file produces only 1-2 chunks, the chunking algorithm is broken.

### Search Quality Benchmarks

| Metric | Poor Implementation | Optimal Implementation |
|--------|---------------------|------------------------|
| **Total Chunks (15 files)** | 15-30 | 300-400 |
| **Chunks per File** | 1-2 | 20-30 |
| **Confidence Score** | 30-40% | 50-60% |
| **Relevance Score** | 30-40% | 50-60% |
| **Chapter Titles** | "Unknown Chapter" | Proper titles |

### Real-World Test Results

**Query:** "What is ROS 2?"

| Implementation | Confidence | Top Relevance | Chapter Titles |
|----------------|------------|---------------|----------------|
| **Before Fixes** | 34% | 36% | "Unknown Chapter" |
| **After Fixes** | 56% | 57% | "Chapter 1: Introduction to Isaac Sim" |
| **Improvement** | **+64%** | **+58%** | **Fixed** |

---

## ✅ Quality Validation Checklist

After indexing, run these checks:

### 1. Chunk Count Validation
```bash
cd backend
python inspect_qdrant.py
```
- **Expected:** 300-400 chunks for ~15 MDX files
- **Red Flag:** < 50 chunks indicates broken chunking

### 2. Field Structure Validation
```bash
python inspect_qdrant.py | head -30
```
- **Check:** `chapter_title: "Chapter 1: ..."`
- **Red Flag:** `chapter_title: NOT FOUND`

### 3. Search Quality Test
```bash
curl -X POST https://your-backend.com/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?"}'
```
- **Expected:** confidence ≥ 50%, relevance ≥ 50%
- **Expected:** Proper chapter titles in sources
- **Red Flag:** confidence < 40%, "Unknown Chapter"

### 4. Individual File Chunking Test
```bash
node test_chunking_fixed.js
```
- **Expected:** 1800-word file → 20+ chunks
- **Red Flag:** Any file producing < 5 chunks

---

## 🚨 Common Issues & Debugging

### Issue 1: Only 1 chunk per file

**Symptoms:**
- Total chunks ≈ number of files (e.g., 15 chunks from 15 files)
- Confidence scores < 40%
- Generic, unhelpful answers

**Root Cause:**
Code block placeholders not preserving double newlines

**Debug:**
```bash
node backend/test_chunking_debug.js
```
Look for: `Paragraphs: 1` ← This is the problem

**Fix:**
Line 296 in `index.js`: Use `\n\n__CODE_BLOCK_${i}__\n\n`

---

### Issue 2: "Unknown Chapter" in all search results

**Symptoms:**
- All sources show "Unknown Chapter"
- Actual chapter titles not displayed
- Poor user experience

**Root Cause:**
Payload uses `title` instead of `chapter_title`

**Debug:**
```bash
python backend/inspect_qdrant.py | grep "chapter_title"
```
Should show: `chapter_title: "Chapter 1: ..."`
NOT: `chapter_title: NOT FOUND`

**Fix:**
Line 247 in `index.js`: Use `chapter_title: title`

---

### Issue 3: Low confidence/relevance scores

**Symptoms:**
- Confidence < 40%
- Relevance < 40%
- Vague, generic answers

**Root Causes:**
1. Poor chunking (see Issue 1)
2. Insufficient chunks (need 300-400 total)
3. Code-heavy chunks dominating text chunks

**Debug:**
```bash
# Check chunk types
python backend/inspect_qdrant.py

# Verify text/code balance
# Expected: ~50-60% text chunks, ~40-50% code chunks
```

**Fix:**
- Ensure chunking creates 20-30 chunks per file
- Verify chunk_size=500 is appropriate for your content
- Check that both text and code chunks are created

---

## 🔧 Configuration Tuning

### Chunk Size Optimization

| Content Type | Recommended chunk_size | Rationale |
|--------------|------------------------|-----------|
| **Text-heavy** (tutorials, explanations) | 500-700 words | Preserve semantic context |
| **Code-heavy** (API docs, examples) | 300-500 words | Smaller context windows |
| **Mixed** (typical robotics chapters) | 500 words | ✅ Default (optimal) |

### Chunk Overlap Optimization

| Use Case | Recommended chunk_overlap | Rationale |
|----------|---------------------------|-----------|
| **Continuous narrative** | 50-100 words | Preserve story flow |
| **Reference documentation** | 25-50 words | Minimize duplication |
| **Code examples** | 50 words | ✅ Default (optimal) |

---

## 📈 Continuous Monitoring

### Production Health Checks

Run after every content update:

```bash
# 1. Verify chunk count
python backend/inspect_qdrant.py | grep "Total points"
# Expected: 300-400 (for ~15 files)

# 2. Sample search quality
curl -X POST https://your-backend.com/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query":"test query"}' | jq '.confidence'
# Expected: ≥ 0.50

# 3. Verify chapter titles
curl -X POST https://your-backend.com/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query":"test query"}' | jq '.sources[0].chapter_title'
# Expected: "Chapter N: ..." (not "Unknown Chapter")
```

### Alert Thresholds

| Metric | Warning | Critical |
|--------|---------|----------|
| Total chunks | < 200 | < 100 |
| Average confidence | < 45% | < 35% |
| "Unknown Chapter" rate | > 10% | > 25% |

---

## 🎯 Summary: The Two Critical Rules

### Rule 1: Always Preserve Paragraph Boundaries
```javascript
textContent.replace(block, `\n\n__CODE_BLOCK_${i}__\n\n`)  // ✅
```

### Rule 2: Always Use 'chapter_title' Field
```javascript
payload: { chapter_title: title }  // ✅
```

**Impact:** These two simple fixes improved search quality by 64% and fixed chapter title display.

---

## 📚 References

- **Implementation:** `.claude/skills/content-indexer/index.js`
- **Backend Contract:** `backend/app/services/vector_service.py`
- **Testing Tools:** `backend/test_chunking_fixed.js`, `backend/inspect_qdrant.py`
- **Performance Metrics:** `backend/chatbot_improvement_results.md`

---

**Last Updated:** 2025-12-17
**Verified Version:** content-indexer v1.0.0
**Test Environment:** 15 MDX files, ~35,000 words total
