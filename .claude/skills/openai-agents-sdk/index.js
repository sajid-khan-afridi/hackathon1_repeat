const { OpenAI } = require('openai');
const { QdrantClient } = require('qdrant-js');
const { v4: uuidv4 } = require('uuid');

class OpenAIAgentsSDK {
  constructor(apiKey, qdrantUrl = 'http://localhost:6333', qdrantApiKey = null) {
    this.openai = new OpenAI({ apiKey });
    this.qdrant = new QdrantClient({
      url: qdrantUrl,
      apiKey: qdrantApiKey
    });
    this.rateLimitStore = new Map();
    this.collectionName = 'physical_ai_textbook';
  }

  /**
   * Create a Physical AI Tutor agent with RAG capabilities
   */
  createPhysicalAITutor(config = {}) {
    const defaultConfig = {
      name: 'PhysicalAI-Tutor',
      instructions: `You are a helpful tutor for a Physical AI & Humanoid Robotics course.
You answer questions based ONLY on the provided textbook content.
If the answer is not in the context, say "I don't have information about that in the textbook."
Always cite the chapter/section when answering.
If the user has selected specific text, prioritize answering about that selection.`,
      model: 'gpt-4o-mini',
      tools: ['search_textbook', 'get_chapter_summary', 'suggest_related_topics'],
      ...config
    };

    return {
      name: defaultConfig.name,
      instructions: defaultConfig.instructions,
      model: defaultConfig.model,
      tools: this.createToolDefinitions(),
      chat: async (query, context = null, chatHistory = []) => {
        return this.handleChat(query, context, chatHistory, defaultConfig);
      }
    };
  }

  /**
   * Define tool schemas for the agent
   */
  createToolDefinitions() {
    return {
      search_textbook: {
        description: 'Search the Physical AI textbook for relevant information',
        parameters: {
          type: 'object',
          properties: {
            query: {
              type: 'string',
              description: 'Search query to find relevant textbook content'
            },
            top_k: {
              type: 'number',
              description: 'Number of results to return',
              default: 5
            }
          },
          required: ['query']
        }
      },
      get_chapter_summary: {
        description: 'Get summary and metadata for a specific chapter',
        parameters: {
          type: 'object',
          properties: {
            chapter_id: {
              type: 'string',
              description: 'Chapter identifier (e.g., "chapter-1", "chapter-2")'
            }
          },
          required: ['chapter_id']
        }
      },
      suggest_related_topics: {
        description: 'Suggest related topics based on current discussion',
        parameters: {
          type: 'object',
          properties: {
            current_topic: {
              type: 'string',
              description: 'Current topic being discussed'
            }
          },
          required: ['current_topic']
        }
      }
    };
  }

  /**
   * Handle chat with streaming response
   */
  async handleChat(query, context, chatHistory, config) {
    const userId = this.generateUserId();

    // Check rate limits
    if (!this.checkRateLimit(userId)) {
      throw new Error('Rate limit exceeded. Please try again later.');
    }

    try {
      // Prepare messages
      const messages = this.prepareMessages(query, context, chatHistory, config);

      // Create streaming response
      const stream = await this.openai.chat.completions.create({
        model: config.model,
        messages,
        tools: Object.values(config.tools).map(tool => ({
          type: 'function',
          function: tool
        })),
        stream: true,
        max_tokens: 10000
      });

      return this.handleStreamResponse(stream, query, context, config);
    } catch (error) {
      console.error('Chat error:', error);
      throw error;
    }
  }

  /**
   * Prepare messages for OpenAI API
   */
  prepareMessages(query, context, chatHistory, config) {
    const messages = [
      {
        role: 'system',
        content: config.instructions + (context ?
          `\n\nThe user has selected the following text for their question: ${context}` : '')
      }
    ];

    // Add chat history
    messages.push(...chatHistory);

    // Add current query
    messages.push({
      role: 'user',
      content: query
    });

    return messages;
  }

  /**
   * Handle streaming response with tool calls
   */
  async handleStreamResponse(stream, originalQuery, context, config) {
    const response = {
      answer: '',
      sources: [],
      follow_up_questions: [],
      token_usage: {
        prompt_tokens: 0,
        completion_tokens: 0,
        total_tokens: 0
      }
    };

    let currentToolCall = null;
    let toolResults = [];

    for await (const chunk of stream) {
      const delta = chunk.choices[0]?.delta;

      if (delta.tool_calls) {
        for (const toolCall of delta.tool_calls) {
          if (toolCall.function) {
            if (!currentToolCall) {
              currentToolCall = {
                id: toolCall.id || uuidv4(),
                name: toolCall.function.name,
                arguments: toolCall.function.arguments || ''
              };
            } else {
              currentToolCall.arguments += toolCall.function.arguments || '';
            }
          }
        }
      } else if (delta.content) {
        response.answer += delta.content;
      }

      // Check if we need to execute a tool call
      if (currentToolCall && currentToolCall.arguments &&
          currentToolCall.arguments.endsWith('}')) {
        try {
          const toolArgs = JSON.parse(currentToolCall.arguments);
          const result = await this.executeToolCall(
            currentToolCall.name,
            toolArgs,
            originalQuery,
            context
          );
          toolResults.push(result);
        } catch (error) {
          console.error('Tool execution error:', error);
        }
        currentToolCall = null;
      }
    }

    // Process tool results
    if (toolResults.length > 0) {
      response.sources = this.extractSources(toolResults);
      response.follow_up_questions = this.generateFollowUpQuestions(
        toolResults,
        originalQuery
      );
    }

    return response;
  }

  /**
   * Execute tool calls
   */
  async executeToolCall(toolName, args, originalQuery, context) {
    switch (toolName) {
      case 'search_textbook':
        return await this.searchTextbook(args.query, args.top_k, context);

      case 'get_chapter_summary':
        return await this.getChapterSummary(args.chapter_id);

      case 'suggest_related_topics':
        return await this.suggestRelatedTopics(args.current_topic);

      default:
        throw new Error(`Unknown tool: ${toolName}`);
    }
  }

  /**
   * Search textbook using RAG with Qdrant
   */
  async searchTextbook(query, topK = 5, context = null) {
    try {
      // Generate embedding for the query
      const embedding = await this.openai.embeddings.create({
        model: 'text-embedding-3-small',
        input: context ? `${context} ${query}` : query
      });

      // Search Qdrant
      const searchResults = await this.qdrant.search(this.collectionName, {
        vector: embedding.data[0].embedding,
        limit: topK,
        with_payload: true
      });

      return {
        results: searchResults.map(result => ({
          text: result.payload.text,
          chapter: result.payload.chapter,
          section: result.payload.section,
          score: result.score
        }))
      };
    } catch (error) {
      console.error('Search error:', error);
      return { results: [] };
    }
  }

  /**
   * Get chapter summary
   */
  async getChapterSummary(chapterId) {
    try {
      const result = await this.qdrant.scroll(this.collectionName, {
        filter: {
          must: [
            { key: 'type', match: { value: 'chapter_metadata' } },
            { key: 'chapter_id', match: { value: chapterId } }
          ]
        },
        limit: 1
      });

      if (result.points.length === 0) {
        return { error: 'Chapter not found' };
      }

      const metadata = result.points[0].payload;
      return {
        title: metadata.title,
        objectives: metadata.objectives || [],
        key_topics: metadata.key_topics || [],
        prerequisites: metadata.prerequisites || []
      };
    } catch (error) {
      console.error('Chapter summary error:', error);
      return { error: 'Failed to retrieve chapter summary' };
    }
  }

  /**
   * Suggest related topics
   */
  async suggestRelatedTopics(currentTopic) {
    try {
      const embedding = await this.openai.embeddings.create({
        model: 'text-embedding-3-small',
        input: currentTopic
      });

      const results = await this.qdrant.search(this.collectionName, {
        vector: embedding.data[0].embedding,
        limit: 10,
        with_payload: true,
        filter: {
          must: [
            { key: 'type', match: { value: 'topic_index' } }
          ]
        }
      });

      return {
        topics: results.slice(0, 5).map(result => ({
          topic: result.payload.topic,
          chapter: result.payload.chapter,
          relevance: result.score
        }))
      };
    } catch (error) {
      console.error('Related topics error:', error);
      return { topics: [] };
    }
  }

  /**
   * Extract sources from tool results
   */
  extractSources(toolResults) {
    const sources = [];
    toolResults.forEach(result => {
      if (result.results) {
        result.results.forEach(item => {
          sources.push({
            chapter: item.chapter,
            section: item.section,
            relevance_score: item.score
          });
        });
      }
    });
    return sources;
  }

  /**
   * Generate follow-up questions
   */
  generateFollowUpQuestions(toolResults, originalQuery) {
    const questions = [];

    // Extract key topics from search results
    const topics = new Set();
    toolResults.forEach(result => {
      if (result.results) {
        result.results.forEach(item => {
          if (item.section) {
            topics.add(item.section);
          }
        });
      }
    });

    // Generate questions based on topics
    const topicArray = Array.from(topics).slice(0, 3);
    topicArray.forEach(topic => {
      questions.push(`Can you explain more about ${topic}?`);
    });

    if (questions.length === 0) {
      questions.push(
        'What are the key concepts in this area?',
        'How does this relate to other topics in the course?',
        'Can you provide examples of this concept?'
      );
    }

    return questions;
  }

  /**
   * Check rate limiting
   */
  checkRateLimit(userId) {
    const now = Date.now();
    const windowMs = 60000; // 1 minute
    const maxRequests = 60;
    const maxTokens = 10000;

    if (!this.rateLimitStore.has(userId)) {
      this.rateLimitStore.set(userId, {
        requests: 0,
        tokens: 0,
        resetTime: now + windowMs
      });
    }

    const userLimit = this.rateLimitStore.get(userId);

    if (now > userLimit.resetTime) {
      userLimit.requests = 0;
      userLimit.tokens = 0;
      userLimit.resetTime = now + windowMs;
    }

    return userLimit.requests < maxRequests && userLimit.tokens < maxTokens;
  }

  /**
   * Generate a user ID for rate limiting
   */
  generateUserId() {
    return uuidv4();
  }

  /**
   * Initialize Qdrant collection
   */
  async initializeCollection() {
    try {
      const collections = await this.qdrant.getCollections();
      const exists = collections.collections.some(
        collection => collection.name === this.collectionName
      );

      if (!exists) {
        await this.qdrant.createCollection(this.collectionName, {
          vectors: {
            size: 1536, // OpenAI embedding dimension
            distance: 'Cosine'
          }
        });
        console.log(`Created collection: ${this.collectionName}`);
      }
    } catch (error) {
      console.error('Collection initialization error:', error);
    }
  }
}

module.exports = { OpenAIAgentsSDK };