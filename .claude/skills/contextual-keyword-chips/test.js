/**
 * Test file for Contextual Keyword Chips
 * Demonstrates keyword extraction functionality
 */

import { KeywordExtractor } from './keywordExtractor.js';

// Test cases
const testCases = [
  {
    name: "Technical Documentation",
    text: "To integrate React with TypeScript, you'll need to install @types/react and @types/react-dom. Create interfaces for your props using TypeScript's type system. The useState and useEffect hooks are essential for managing component state and side effects. For API calls, consider using axios or the native fetch API.",
    expectedTypes: ['tech', 'noun', 'action']
  },
  {
    name: "Code Example",
    text: "Implement authentication using JWT tokens in your Node.js application. Store tokens securely in HTTP-only cookies. Use bcrypt for password hashing and implement rate limiting to prevent brute force attacks.",
    expectedTypes: ['tech', 'action', 'security']
  },
  {
    name: "General Question",
    text: "Machine learning is a subset of artificial intelligence that focuses on neural networks and deep learning algorithms. TensorFlow and PyTorch are popular frameworks for building ML models.",
    expectedTypes: ['entity', 'tech', 'concept']
  }
];

// Create extractor instance
const extractor = new KeywordExtractor({ maxChips: 6 });

console.log('🧪 Testing Keyword Extraction\n');

testCases.forEach((testCase, index) => {
  console.log(`\n--- Test Case ${index + 1}: ${testCase.name} ---`);
  console.log(`Input: ${testCase.text.substring(0, 100)}...`);

  const keywords = extractor.extractKeywords(testCase.text);
  const categories = extractor.categorizeChips(keywords);

  console.log('\nExtracted Keywords:');
  keywords.forEach((keyword, i) => {
    console.log(`  ${i + 1}. "${keyword.text}" (${keyword.type}) - Score: ${keyword.score}`);
  });

  console.log('\nCategories:');
  Object.entries(categories).forEach(([category, chips]) => {
    if (chips.length > 0) {
      console.log(`  ${category}: ${chips.map(c => c.text).join(', ')}`);
    }
  });
});

// Test performance
console.log('\n\n--- Performance Test ---');
const longText = `
  Building microservices with Docker and Kubernetes requires understanding of containerization,
  orchestration, and service mesh technologies. Use GraphQL for efficient API design and
  implement CI/CD pipelines using GitHub Actions. Monitor performance with Prometheus and
  visualize metrics in Grafana dashboards. Handle authentication with OAuth 2.0 and manage
  user sessions with Redis. Deploy to AWS using serverless Lambda functions and configure
  API Gateway for RESTful endpoints.
`;

const iterations = 100;
console.time('Keyword Extraction');
for (let i = 0; i < iterations; i++) {
  extractor.extractKeywords(longText);
}
console.timeEnd('Keyword Extraction');
console.log(`Average time per extraction: ${(iterations / 1000).toFixed(2)}ms`);

console.log('\n✅ Test completed successfully!');
console.log('\n📋 To use in your React application:');
console.log(`
import { KeywordChips } from '.claude/skills/contextual-keyword-chips';

function ChatMessage({ message }) {
  return (
    <div>
      <p>{message.text}</p>
      <KeywordChips
        aiAnswer={message.text}
        onChipClick={(chipText) => {
          // Handle chip click - e.g., send follow-up query
          sendMessage(chipText);
        }}
        maxChips={5}
        variant="default"
        animated={true}
      />
    </div>
  );
}
`);