# Qdrant VectorStore Skill

A Claude Code skill for managing Qdrant Cloud vector operations, designed to handle document embedding storage and semantic search for RAG (Retrieval-Augmented Generation) chatbots.

## Features

- **Collection Management**: Create and manage Qdrant collections
- **Document Storage**: Embed and store documents with automatic chunking
- **Semantic Search**: Search documents using vector similarity
- **Error Handling**: Built-in retry logic with exponential backoff
- **Logging**: Comprehensive operation logging

## Setup

### Prerequisites

1. **Qdrant Cloud Account**: Sign up at [Qdrant Cloud](https://qdrant.tech/cloud/)
2. **OpenAI API Key**: Get an API key from [OpenAI](https://platform.openai.com/)

### Installation

```bash
cd .claude/skills/qdrant-vectorstore
npm install
```

### Environment Variables

Create a `.env` file in the skill directory:

```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
OPENAI_API_KEY=your-openai-api-key
```

## Usage

### Create Collection

```bash
/ask Use qdrant-vectorstore to create a collection named "my_documents"
```

### Store Documents

```bash
/ask Use qdrant-vectorstore to upsert these documents into "my_documents":
- "Machine learning is a subset of artificial intelligence..."
- "Deep learning uses neural networks with multiple layers..."
- "Natural language processing helps computers understand human language..."
```

### Search Documents

```bash
/ask Use qdrant-vectorstore to search "my_documents" for "neural networks"
```

## API Reference

### Operations

#### create_collection
Creates a new Qdrant collection for storing vectors.

**Parameters:**
- `collection_name` (string): Name of the collection to create

**Returns:**
```json
{
  "collection_id": "collection_name"
}
```

#### upsert
Embeds and stores documents in the specified collection.

**Parameters:**
- `collection_name` (string): Target collection name
- `documents` (string[]): Array of document texts to store

**Returns:**
```json
{
  "inserted_count": 15
}
```

#### search
Performs semantic search in the collection.

**Parameters:**
- `collection_name` (string): Collection to search in
- `query` (string): Search query text
- `top_k` (number, optional): Number of results to return (default: 5)

**Returns:**
```json
[
  {
    "text": "Deep learning uses neural networks...",
    "score": 0.89,
    "metadata": {
      "document_index": 1,
      "chunk_index": 0,
      "timestamp": "2024-01-01T00:00:00.000Z"
    }
  }
]
```

## Configuration

### Embedding Model
- **Model**: text-embedding-3-small (OpenAI)
- **Dimensions**: 1536
- **Distance Metric**: Cosine similarity

### Document Processing
- **Chunk Size**: 8000 characters per chunk
- **Chunking Strategy**: Breaks at sentence boundaries when possible
- **Batch Size**: 100 documents per upsert batch

### Rate Limiting
- **Max Retries**: 3 attempts
- **Backoff Strategy**: Exponential (1s, 2s, 4s)
- **Rate Limit Handling**: Automatic retry on 429 errors

## Example Workflow

```bash
# 1. Create a collection for documentation
/ask Use qdrant-vectorstore to create collection "tech_docs"

# 2. Add documentation
/ask Use qdrant-vectorstore to upsert into "tech_docs":
- "React is a JavaScript library for building user interfaces..."
- "Node.js is a JavaScript runtime built on Chrome's V8 engine..."
- "MongoDB is a document-oriented NoSQL database..."

# 3. Search for relevant information
/ask Use qdrant-vectorstore to search "tech_docs" for "JavaScript runtime"
```

## Testing

Run the test suite:

```bash
npm test
```

Note: Tests require valid API credentials to run against actual services.

## Error Handling

The skill handles various error scenarios:

- **Network Timeouts**: Automatic retry with exponential backoff
- **Rate Limits**: Respects API rate limits with delays
- **Collection Validation**: Checks if collection exists before operations
- **Input Validation**: Validates required parameters
- **Graceful Degradation**: Detailed error messages for debugging

## Logging

All operations are logged with:
- Timestamp
- Operation type
- Collection name
- Success/failure status
- Error details (if applicable)

## Best Practices

1. **Document Chunking**: Long documents are automatically chunked at sentence boundaries
2. **Batch Operations**: Documents are upserted in batches to avoid payload limits
3. **Metadata**: Each document chunk includes document index, chunk index, and timestamp
4. **Idempotency**: Create collection operation is idempotent (returns existing collection)

## Troubleshooting

### Common Issues

1. **"Collection does not exist"**: Run create_collection operation first
2. **Rate limit errors**: The skill automatically retries, but consider reducing request frequency
3. **Timeout errors**: Check network connectivity and API key validity
4. **Embedding failures**: Verify OpenAI API key and quota

### Debug Mode

Set `DEBUG=1` environment variable to see detailed logs:

```bash
DEBUG=1 /ask Use qdrant-vectorstore to search "collection" "query"
```

## Limitations

- Qdrant Cloud Free Tier has usage limits
- OpenAI API has rate limits and costs
- Document size limited by embedding model (effectively handled by chunking)
- Search quality depends on document content and query specificity