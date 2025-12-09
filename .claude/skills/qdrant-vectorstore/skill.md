# Qdrant VectorStore Skill

## Skill Information
**Name:** qdrant-vectorstore
**Description:** Manage Qdrant Cloud vector operations for document embedding storage and semantic search
**Version:** 1.0.0
**Author:** Claude Code

## Purpose
Handle document embedding storage and semantic search for RAG chatbot using Qdrant Cloud.

## Inputs
- **operation** (required): "create_collection" | "upsert" | "search"
- **collection_name** (required): string - Name of the Qdrant collection
- **documents** (optional): string[] - Array of documents to embed (for upsert operation)
- **query** (optional): string - Search query text (for search operation)
- **top_k** (optional): number - Number of results to return (default: 5)

## Outputs
- **For create_collection**: `{ collection_id: string }`
- **For upsert**: `{ inserted_count: number }`
- **For search**: Array of `{ text: string, score: number, metadata: object }`

## Implementation Requirements
- **Platform**: Qdrant Cloud Free Tier endpoint
- **Embedding Model**: text-embedding-3-small (OpenAI)
- **Vector Dimension**: 1536
- **Distance Metric**: Cosine

## Environment Variables
- `QDRANT_URL`: Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `OPENAI_API_KEY`: OpenAI API key for embeddings

## Error Handling
- Retry on rate limits with exponential backoff
- Validate collection exists before operations
- Log all operations for debugging
- Handle network timeouts gracefully

## Usage Examples

### Create Collection
```json
{
  "operation": "create_collection",
  "collection_name": "my_documents"
}
```

### Upsert Documents
```json
{
  "operation": "upsert",
  "collection_name": "my_documents",
  "documents": [
    "Document 1 text content",
    "Document 2 text content"
  ]
}
```

### Search Documents
```json
{
  "operation": "search",
  "collection_name": "my_documents",
  "query": "What is the meaning of life?",
  "top_k": 3
}
```

## Dependencies
- @qdrant/js-client-rest
- openai
- dotenv

## Implementation Notes
- Documents are automatically chunked if too long for embedding model
- Each document gets a unique ID and timestamp metadata
- Search results are ranked by similarity score (0-1)
- All operations are logged with timestamps and success/failure status