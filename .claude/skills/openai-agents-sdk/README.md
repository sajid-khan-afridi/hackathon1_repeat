# OpenAI Agents SDK Skill

This skill integrates OpenAI's Agents SDK to build an intelligent RAG (Retrieval-Augmented Generation) chatbot for the Physical AI textbook content.

## Features

- **RAG-powered Q&A**: Search and retrieve relevant textbook content using Qdrant vector database
- **Context-aware responses**: Handles selected text and chat history
- **Streaming responses**: Real-time response generation with typing indicators
- **Rate limiting**: Built-in protection against API abuse
- **Tool-based architecture**: Modular tools for different capabilities

## Installation

```bash
npm install openai @openai/agents qdrant-js uuid
```

## Environment Variables

```bash
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=http://localhost:6333  # Optional
QDRANT_API_KEY=your_qdrant_key    # Optional
```

## Quick Start

```javascript
const { OpenAIAgentsSDK } = require('./index');

// Initialize SDK
const sdk = new OpenAIAgentsSDK(process.env.OPENAI_API_KEY);

// Create Physical AI Tutor
const tutor = sdk.createPhysicalAITutor();

// Chat with the tutor
const response = await tutor.chat(
  'What is forward kinematics?',
  null, // optional selected text context
  []    // optional chat history
);

console.log(response.answer);
console.log('Sources:', response.sources);
console.log('Follow-up questions:', response.follow_up_questions);
```

## Agent Configuration

```javascript
const customTutor = sdk.createPhysicalAITutor({
  name: 'Advanced-Robotics-Tutor',
  instructions: `You are an expert in robotics and physical AI.
  Provide detailed explanations with mathematical formulations.`,
  model: 'gpt-4o',  // Use more capable model
  tools: ['search_textbook', 'get_chapter_summary']
});
```

## Available Tools

### 1. search_textbook

Search the Physical AI textbook for relevant information using semantic search.

```javascript
const results = await sdk.searchTextbook(
  'query string',
  5,  // top_k results
  'optional context'  // selected text
);
```

### 2. get_chapter_summary

Retrieve metadata and summary for a specific chapter.

```javascript
const summary = await sdk.getChapterSummary('chapter-3');
console.log(summary.title);
console.log(summary.objectives);
console.log(summary.key_topics);
```

### 3. suggest_related_topics

Find related topics based on current discussion.

```javascript
const related = await sdk.suggestRelatedTopics('robot dynamics');
console.log(related.topics);
```

## Response Format

```javascript
{
  answer: "The answer to your question...",
  sources: [
    {
      chapter: "Chapter 2: Kinematics",
      section: "2.3 Forward Kinematics",
      relevance_score: 0.92
    }
  ],
  follow_up_questions: [
    "Can you explain inverse kinematics?",
    "How does this apply to serial manipulators?"
  ],
  token_usage: {
    prompt_tokens: 150,
    completion_tokens: 300,
    total_tokens: 450
  }
}
```

## Rate Limiting

- **Requests**: 60 per minute per user
- **Tokens**: 10,000 per request
- **Queue**: Requests are queued during high load

## Streaming Example

```javascript
const response = await tutor.chat(
  'Explain the concepts in detail',
  null,
  []
);

// Process streaming response
for await (const chunk of response.answer) {
  process.stdout.write(chunk);
}
```

## Context Handling

When users select specific text:

```javascript
const selectedText = "The Denavit-Hartenberg convention...";
const response = await tutor.chat(
  'Can you explain this?',
  selectedText,  // Context is prioritized
  []
);
```

## Error Handling

```javascript
try {
  const response = await tutor.chat('Your question', null, []);
} catch (error) {
  if (error.message.includes('Rate limit')) {
    console.log('Please wait before making another request');
  } else {
    console.error('Chat error:', error);
  }
}
```

## Qdrant Integration

The skill uses Qdrant for vector storage and retrieval:

1. **Collection**: `physical_ai_textbook`
2. **Embedding Model**: `text-embedding-3-small` (1536 dimensions)
3. **Distance Metric**: Cosine similarity

## Data Structure

Each point in Qdrant should have:

```javascript
{
  text: "Text content...",
  chapter: "Chapter 1",
  section: "1.2 Introduction",
  type: "textbook_content",  // or "chapter_metadata", "topic_index"
  chapter_id: "chapter-1",   // for metadata
  topic: "Robot Basics",      // for topic index
  objectives: [...],          // for chapter metadata
  key_topics: [...],          // for chapter metadata
  prerequisites: [...]        // for chapter metadata
}
```

## Best Practices

1. **Chunk Size**: Keep text chunks between 200-500 words
2. **Overlap**: Add 10% overlap between chunks for context
3. **Metadata**: Include rich metadata for better filtering
4. **Caching**: Cache frequent queries to reduce costs
5. **Monitoring**: Track token usage and response times

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## License

MIT License - see LICENSE file for details.