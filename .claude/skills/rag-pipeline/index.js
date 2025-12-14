#!/usr/bin/env node

import OpenAI from 'openai';
import { QdrantClient } from '@qdrant/js-client-rest';
import { Pool } from 'pg';
import crypto from 'crypto';

/**
 * RAG Pipeline Skill
 * End-to-end RAG orchestration: vector search, context retrieval, AI response generation
 */
class RagPipeline {
  constructor(options = {}) {
    this.collectionName = options.collectionName || 'robotics_textbook';
    this.topK = options.topK || 5;
    this.embeddingModel = process.env.EMBEDDING_MODEL || 'text-embedding-3-small';
    this.chatModel = options.chatModel || 'gpt-4o-mini';

    // Initialize clients
    this.openai = null;
    this.qdrant = null;
    this.pool = null;
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

    // Initialize database connection for chat history
    if (process.env.DATABASE_URL) {
      this.pool = new Pool({
        connectionString: process.env.DATABASE_URL,
        max: parseInt(process.env.DATABASE_POOL_MAX || '10'),
        ssl: process.env.DATABASE_URL.includes('neon.tech')
          ? { rejectUnauthorized: false }
          : false
      });
    }
  }

  /**
   * Disconnect from services
   */
  async disconnect() {
    if (this.pool) {
      await this.pool.end();
      this.pool = null;
    }
  }

  /**
   * Process a user query through the RAG pipeline
   */
  async query(options = {}) {
    const {
      userQuery,
      userId,
      topK = this.topK,
      filters = {},
      includeChatHistory = true,
      streamResponse = false,
      chatId = crypto.randomUUID()
    } = options;

    if (!userQuery) {
      throw new Error('userQuery is required');
    }

    await this.initialize();

    try {
      const tokensUsed = { retrieval: 0, generation: 0, total: 0 };

      // Step 1: Retrieve chat history (if enabled and user provided)
      let chatHistory = [];
      if (includeChatHistory && userId && this.pool) {
        chatHistory = await this.getChatHistory(userId, chatId);
      }

      // Step 2: Generate embedding for the query
      const queryEmbedding = await this.generateEmbedding(userQuery);
      tokensUsed.retrieval = queryEmbedding.tokensUsed;

      // Step 3: Search Qdrant for relevant context
      const searchResults = await this.searchContext(queryEmbedding.embedding, topK, filters);

      // Step 4: Build context and generate AI response
      const context = this.buildContext(searchResults);
      const sources = this.extractSources(searchResults);

      const response = await this.generateResponse({
        userQuery,
        context,
        chatHistory,
        streamResponse
      });

      tokensUsed.generation = response.tokensUsed;
      tokensUsed.total = tokensUsed.retrieval + tokensUsed.generation;

      // Step 5: Save to chat history (if enabled)
      if (userId && this.pool) {
        await this.saveChatExchange(userId, chatId, userQuery, response.answer, sources, tokensUsed);
      }

      // Step 6: Calculate confidence score
      const confidence = this.calculateConfidence(searchResults);

      return {
        success: true,
        answer: response.answer,
        sources,
        confidence,
        chat_id: chatId,
        tokens_used: tokensUsed
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Generate embedding for a query
   */
  async generateEmbedding(text) {
    const response = await this.openai.embeddings.create({
      model: this.embeddingModel,
      input: text
    });

    return {
      embedding: response.data[0].embedding,
      tokensUsed: response.usage.total_tokens
    };
  }

  /**
   * Search Qdrant for relevant context
   */
  async searchContext(embedding, topK, filters = {}) {
    const searchParams = {
      vector: embedding,
      limit: topK,
      with_payload: true,
      score_threshold: 0.5 // Minimum relevance threshold
    };

    // Build filter conditions
    if (Object.keys(filters).length > 0) {
      const must = [];

      if (filters.module) {
        must.push({ key: 'module', match: { value: filters.module } });
      }

      if (filters.chapter_id) {
        must.push({ key: 'chapter_id', match: { value: filters.chapter_id } });
      }

      if (filters.tags && filters.tags.length > 0) {
        for (const tag of filters.tags) {
          must.push({ key: 'tags', match: { value: tag } });
        }
      }

      if (must.length > 0) {
        searchParams.filter = { must };
      }
    }

    const results = await this.qdrant.search(this.collectionName, searchParams);
    return results;
  }

  /**
   * Build context string from search results
   */
  buildContext(searchResults) {
    if (searchResults.length === 0) {
      return 'No relevant context found in the textbook.';
    }

    const contextParts = searchResults.map((result, index) => {
      const payload = result.payload;
      return `[Source ${index + 1}: ${payload.title} (${payload.chapter_id})]
${payload.text}`;
    });

    return contextParts.join('\n\n---\n\n');
  }

  /**
   * Extract source information from results
   */
  extractSources(searchResults) {
    return searchResults.map(result => ({
      chapter_id: result.payload.chapter_id,
      title: result.payload.title,
      score: Math.round(result.score * 100) / 100,
      excerpt: result.payload.text.substring(0, 200) + '...'
    }));
  }

  /**
   * Generate AI response using context
   */
  async generateResponse(options = {}) {
    const { userQuery, context, chatHistory, streamResponse } = options;

    const systemPrompt = `You are a helpful robotics instructor assistant for an educational textbook about ROS (Robot Operating System), Isaac Sim, and robotics concepts.

Your role is to:
1. Answer questions accurately based on the provided context from the textbook
2. Explain technical concepts clearly for students at various skill levels
3. Reference specific chapters or sections when relevant
4. Admit when information is not available in the context
5. Provide code examples when helpful (use Python for ROS examples)

Guidelines:
- Be concise but thorough
- Use markdown formatting for code blocks and lists
- If the context doesn't contain relevant information, say so honestly
- Encourage students to explore related topics in the textbook`;

    const messages = [
      { role: 'system', content: systemPrompt },
      { role: 'system', content: `Relevant context from the textbook:\n\n${context}` }
    ];

    // Add chat history
    for (const msg of chatHistory) {
      messages.push({ role: msg.role, content: msg.content });
    }

    // Add current query
    messages.push({ role: 'user', content: userQuery });

    if (streamResponse) {
      // Return streaming response (for real-time UI)
      const stream = await this.openai.chat.completions.create({
        model: this.chatModel,
        messages,
        stream: true
      });

      return {
        stream,
        tokensUsed: 0 // Will be calculated after streaming completes
      };
    }

    const response = await this.openai.chat.completions.create({
      model: this.chatModel,
      messages,
      temperature: 0.7,
      max_tokens: 1500
    });

    return {
      answer: response.choices[0].message.content,
      tokensUsed: response.usage.total_tokens
    };
  }

  /**
   * Get chat history from database
   */
  async getChatHistory(userId, chatId, limit = 10) {
    const result = await this.pool.query(
      `SELECT role, content FROM chat_history
       WHERE user_id = $1 AND chat_id = $2
       ORDER BY created_at DESC
       LIMIT $3`,
      [userId, chatId, limit]
    );

    // Reverse to get chronological order
    return result.rows.reverse();
  }

  /**
   * Save chat exchange to database
   */
  async saveChatExchange(userId, chatId, userQuery, assistantResponse, sources, tokensUsed) {
    // Save user message
    await this.pool.query(
      `INSERT INTO chat_history (user_id, chat_id, role, content, tokens_used)
       VALUES ($1, $2, 'user', $3, $4)`,
      [userId, chatId, userQuery, tokensUsed.retrieval]
    );

    // Save assistant message
    await this.pool.query(
      `INSERT INTO chat_history (user_id, chat_id, role, content, sources, tokens_used)
       VALUES ($1, $2, 'assistant', $3, $4, $5)`,
      [userId, chatId, assistantResponse, JSON.stringify(sources), tokensUsed.generation]
    );
  }

  /**
   * Calculate confidence score based on search results
   */
  calculateConfidence(searchResults) {
    if (searchResults.length === 0) {
      return 0;
    }

    // Average of top 3 scores (or all if less than 3)
    const topScores = searchResults.slice(0, 3).map(r => r.score);
    const avgScore = topScores.reduce((a, b) => a + b, 0) / topScores.length;

    return Math.round(avgScore * 100) / 100;
  }

  /**
   * Get recent chat sessions for a user
   */
  async getUserChatSessions(userId, limit = 10) {
    if (!this.pool) {
      await this.initialize();
    }

    const result = await this.pool.query(
      `SELECT DISTINCT ON (chat_id) chat_id, content as first_message, created_at
       FROM chat_history
       WHERE user_id = $1 AND role = 'user'
       ORDER BY chat_id, created_at ASC
       LIMIT $2`,
      [userId, limit]
    );

    await this.disconnect();
    return result.rows;
  }

  /**
   * Continue an existing chat session
   */
  async continueChat(options = {}) {
    const { chatId, userId, userQuery } = options;

    if (!chatId || !userQuery) {
      throw new Error('chatId and userQuery are required');
    }

    return this.query({
      ...options,
      chatId,
      includeChatHistory: true
    });
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: rag-pipeline <operation> [options]');
    console.error('Operations: query, continue, sessions');
    console.error('');
    console.error('Examples:');
    console.error('  rag-pipeline query --question "How do I create a ROS publisher?"');
    console.error('  rag-pipeline query --question "What is Isaac Sim?" --module 2');
    console.error('  rag-pipeline continue --chat_id "uuid" --question "Tell me more"');
    console.error('  rag-pipeline sessions --user_id "uuid"');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const pipeline = new RagPipeline({
    collectionName: options.collection || 'robotics_textbook',
    topK: parseInt(options.top_k || '5'),
    chatModel: options.model || 'gpt-4o-mini'
  });

  try {
    let result;
    switch (operation) {
      case 'query':
        result = await pipeline.query({
          userQuery: options.question,
          userId: options.user_id,
          topK: parseInt(options.top_k || '5'),
          filters: {
            module: options.module ? parseInt(options.module) : undefined,
            chapter_id: options.chapter
          },
          includeChatHistory: options.history !== 'false'
        });
        break;

      case 'continue':
        result = await pipeline.continueChat({
          chatId: options.chat_id,
          userId: options.user_id,
          userQuery: options.question
        });
        break;

      case 'sessions':
        result = await pipeline.getUserChatSessions(
          options.user_id,
          parseInt(options.limit || '10')
        );
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
export default RagPipeline;
export { RagPipeline };

// Run CLI if called directly
const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('rag-pipeline/index.js');
if (isMainModule) {
  main().catch(console.error);
}
