import OpenAI from 'openai';

export interface AgentConfig {
  name?: string;
  instructions?: string;
  model?: string;
  tools?: string[];
}

export interface ChatMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
}

export interface SearchResult {
  text: string;
  chapter: string;
  section: string;
  score: number;
}

export interface ChapterSummary {
  title?: string;
  objectives?: string[];
  key_topics?: string[];
  prerequisites?: string[];
  error?: string;
}

export interface RelatedTopic {
  topic: string;
  chapter: string;
  relevance: number;
}

export interface Source {
  chapter: string;
  section: string;
  relevance_score: number;
}

export interface TokenUsage {
  prompt_tokens: number;
  completion_tokens: number;
  total_tokens: number;
}

export interface ChatResponse {
  answer: string;
  sources: Source[];
  follow_up_questions: string[];
  token_usage: TokenUsage;
}

export interface ToolResult {
  results?: SearchResult[];
  topics?: RelatedTopic[];
  title?: string;
  objectives?: string[];
  key_topics?: string[];
  prerequisites?: string[];
  error?: string;
}

export interface PhysicalAITutor {
  name: string;
  instructions: string;
  model: string;
  tools: Record<string, any>;
  chat: (
    query: string,
    context?: string | null,
    chatHistory?: ChatMessage[]
  ) => Promise<ChatResponse>;
}

export class OpenAIAgentsSDK {
  constructor(
    apiKey: string,
    qdrantUrl?: string,
    qdrantApiKey?: string
  );

  /**
   * Create a Physical AI Tutor agent with RAG capabilities
   */
  createPhysicalAITutor(config?: AgentConfig): PhysicalAITutor;

  /**
   * Search textbook using RAG with Qdrant
   */
  searchTextbook(
    query: string,
    topK?: number,
    context?: string | null
  ): Promise<{ results: SearchResult[] }>;

  /**
   * Get chapter summary
   */
  getChapterSummary(chapterId: string): Promise<ChapterSummary>;

  /**
   * Suggest related topics
   */
  suggestRelatedTopics(currentTopic: string): Promise<{ topics: RelatedTopic[] }>;

  /**
   * Initialize Qdrant collection
   */
  initializeCollection(): Promise<void>;
}

export default OpenAIAgentsSDK;