/**
 * Qdrant VectorStore Skill
 * Manages Qdrant Cloud vector operations for RAG chatbot
 */

const { QdrantClient } = require('@qdrant/js-client-rest');
const OpenAI = require('openai');
require('dotenv').config();

// Constants
const VECTOR_SIZE = 1536;
const MAX_RETRIES = 3;
const RETRY_DELAY_BASE = 1000; // 1 second
const MAX_DOCUMENT_LENGTH = 8000; // Chars per chunk

class QdrantVectorStore {
  constructor() {
    // Validate environment variables
    if (!process.env.QDRANT_URL || !process.env.QDRANT_API_KEY || !process.env.OPENAI_API_KEY) {
      throw new Error('Missing required environment variables: QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY');
    }

    this.qdrant = new QdrantClient({
      url: process.env.QDRANT_URL,
      apiKey: process.env.QDRANT_API_KEY
    });

    this.openai = new OpenAI({
      apiKey: process.env.OPENAI_API_KEY
    });
  }

  /**
   * Log operation with timestamp
   */
  log(message, data = {}) {
    const timestamp = new Date().toISOString();
    console.log(`[${timestamp}] QdrantVectorStore: ${message}`, data);
  }

  /**
   * Retry with exponential backoff
   */
  async retryWithBackoff(operation, retries = MAX_RETRIES) {
    for (let i = 0; i < retries; i++) {
      try {
        return await operation();
      } catch (error) {
        if (i === retries - 1) throw error; // Last attempt, throw error

        const delay = RETRY_DELAY_BASE * Math.pow(2, i);
        this.log(`Retry ${i + 1}/${retries} after ${delay}ms`, { error: error.message });

        await this.sleep(delay);
      }
    }
  }

  /**
   * Sleep helper
   */
  sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Check if collection exists
   */
  async collectionExists(collectionName) {
    try {
      const collections = await this.qdrant.getCollections();
      return collections.collections.some(c => c.name === collectionName);
    } catch (error) {
      this.log('Error checking collection existence', { collectionName, error: error.message });
      throw error;
    }
  }

  /**
   * Create a new collection
   */
  async createCollection(collectionName) {
    this.log('Creating collection', { collectionName });

    try {
      // Check if collection already exists
      if (await this.collectionExists(collectionName)) {
        this.log('Collection already exists', { collectionName });
        return { collection_id: collectionName };
      }

      await this.retryWithBackoff(async () => {
        await this.qdrant.createCollection(collectionName, {
          vectors: {
            size: VECTOR_SIZE,
            distance: 'Cosine'
          }
        });
      });

      this.log('Collection created successfully', { collectionName });
      return { collection_id: collectionName };

    } catch (error) {
      this.log('Error creating collection', { collectionName, error: error.message });
      throw error;
    }
  }

  /**
   * Chunk long documents
   */
  chunkDocument(text, maxLength = MAX_DOCUMENT_LENGTH) {
    if (text.length <= maxLength) return [text];

    const chunks = [];
    let start = 0;

    while (start < text.length) {
      let end = start + maxLength;

      // Try to break at sentence boundary
      if (end < text.length) {
        const lastPeriod = text.lastIndexOf('.', end);
        const lastNewline = text.lastIndexOf('\n', end);
        const breakPoint = Math.max(lastPeriod, lastNewline);

        if (breakPoint > start) {
          end = breakPoint + 1;
        }
      }

      chunks.push(text.slice(start, end));
      start = end;
    }

    return chunks;
  }

  /**
   * Generate embeddings for text
   */
  async generateEmbedding(text) {
    return await this.retryWithBackoff(async () => {
      const response = await this.openai.embeddings.create({
        model: 'text-embedding-3-small',
        input: text
      });
      return response.data[0].embedding;
    });
  }

  /**
   * Upsert documents into collection
   */
  async upsert(collectionName, documents) {
    this.log('Upserting documents', { collectionName, count: documents.length });

    try {
      // Validate collection exists
      if (!(await this.collectionExists(collectionName))) {
        throw new Error(`Collection '${collectionName}' does not exist`);
      }

      let totalInserted = 0;
      const points = [];

      for (let i = 0; i < documents.length; i++) {
        const doc = documents[i];
        const chunks = this.chunkDocument(doc);

        for (let j = 0; j < chunks.length; j++) {
          const chunk = chunks[j];
          const embedding = await this.generateEmbedding(chunk);

          points.push({
            id: `${i}_${j}_${Date.now()}`,
            vector: embedding,
            payload: {
              text: chunk,
              document_index: i,
              chunk_index: j,
              timestamp: new Date().toISOString()
            }
          });
        }
      }

      // Upsert in batches to avoid payload limits
      const batchSize = 100;
      for (let i = 0; i < points.length; i += batchSize) {
        const batch = points.slice(i, i + batchSize);

        await this.retryWithBackoff(async () => {
          await this.qdrant.upsert(collectionName, {
            wait: true,
            points: batch
          });
        });

        totalInserted += batch.length;
      }

      this.log('Documents upserted successfully', {
        collectionName,
        totalInserted,
        originalDocuments: documents.length
      });

      return { inserted_count: totalInserted };

    } catch (error) {
      this.log('Error upserting documents', { collectionName, error: error.message });
      throw error;
    }
  }

  /**
   * Search documents
   */
  async search(collectionName, query, top_k = 5) {
    this.log('Searching documents', { collectionName, query, top_k });

    try {
      // Validate collection exists
      if (!(await this.collectionExists(collectionName))) {
        throw new Error(`Collection '${collectionName}' does not exist`);
      }

      // Generate query embedding
      const queryEmbedding = await this.generateEmbedding(query);

      // Search in Qdrant
      const searchResult = await this.retryWithBackoff(async () => {
        return await this.qdrant.search(collectionName, {
          vector: queryEmbedding,
          limit: top_k,
          with_payload: true
        });
      });

      // Format results
      const results = searchResult.map(result => ({
        text: result.payload.text,
        score: result.score,
        metadata: {
          document_index: result.payload.document_index,
          chunk_index: result.payload.chunk_index,
          timestamp: result.payload.timestamp
        }
      }));

      this.log('Search completed', { collectionName, resultsFound: results.length });

      return results;

    } catch (error) {
      this.log('Error searching documents', { collectionName, query, error: error.message });
      throw error;
    }
  }

  /**
   * Main execution handler
   */
  async execute(input) {
    const { operation, collection_name, documents, query, top_k } = input;

    // Validate required fields
    if (!operation || !collection_name) {
      throw new Error('Missing required fields: operation, collection_name');
    }

    switch (operation) {
      case 'create_collection':
        return await this.createCollection(collection_name);

      case 'upsert':
        if (!documents || !Array.isArray(documents)) {
          throw new Error('Missing or invalid documents array for upsert operation');
        }
        return await this.upsert(collection_name, documents);

      case 'search':
        if (!query) {
          throw new Error('Missing query for search operation');
        }
        return await this.search(collection_name, query, top_k || 5);

      default:
        throw new Error(`Unsupported operation: ${operation}`);
    }
  }
}

// Export for use as skill
module.exports = {
  name: 'qdrant-vectorstore',
  description: 'Manage Qdrant Cloud vector operations for RAG chatbot',
  handler: async (input) => {
    const store = new QdrantVectorStore();
    return await store.execute(input);
  }
};