const { OpenAIAgentsSDK } = require('./index');

// Initialize the SDK
const sdk = new OpenAIAgentsSDK(
  process.env.OPENAI_API_KEY,
  process.env.QDRANT_URL || 'http://localhost:6333',
  process.env.QDRANT_API_KEY
);

async function main() {
  // Initialize Qdrant collection
  await sdk.initializeCollection();

  // Create Physical AI Tutor agent
  const tutor = sdk.createPhysicalAITutor();

  // Example 1: Simple question
  console.log('Example 1: Simple question');
  const response1 = await tutor.chat(
    'What is forward kinematics?',
    null,
    []
  );
  console.log('Answer:', response1.answer);
  console.log('Sources:', response1.sources);
  console.log('Follow-up questions:', response1.follow_up_questions);

  // Example 2: Question with context (selected text)
  console.log('\nExample 2: Question with selected text');
  const selectedText = 'The Denavit-Hartenberg convention provides a systematic way to assign coordinate frames to robot joints.';
  const response2 = await tutor.chat(
    'Can you explain this in more detail?',
    selectedText,
    []
  );
  console.log('Answer:', response2.answer);

  // Example 3: Chat with history
  console.log('\nExample 3: Chat with history');
  const chatHistory = [
    { role: 'user', content: 'What are degrees of freedom in robotics?' },
    { role: 'assistant', content: 'Degrees of freedom (DOF) in robotics refers to the number of independent movements a robot can make...' }
  ];
  const response3 = await tutor.chat(
    'How does DOF relate to configuration space?',
    null,
    chatHistory
  );
  console.log('Answer:', response3.answer);

  // Example 4: Streaming response
  console.log('\nExample 4: Streaming response');
  const response4 = await tutor.chat(
    'Explain the concept of workspace in humanoid robots',
    null,
    []
  );

  // Process streaming response
  console.log('Response:');
  for await (const chunk of response4.answer) {
    process.stdout.write(chunk);
  }
  console.log('\nSources:', response4.sources);
}

// Example tool usage
async function exampleToolUsage() {
  console.log('\n=== Direct Tool Usage Examples ===');

  // Search textbook
  const searchResults = await sdk.searchTextbook(
    'inverse kinematics algorithms',
    5,
    null
  );
  console.log('Search results:', searchResults.results.slice(0, 2));

  // Get chapter summary
  const chapterSummary = await sdk.getChapterSummary('chapter-3');
  console.log('Chapter summary:', chapterSummary);

  // Suggest related topics
  const relatedTopics = await sdk.suggestRelatedTopics('robot dynamics');
  console.log('Related topics:', relatedTopics.topics.slice(0, 3));
}

// Example error handling
async function exampleErrorHandling() {
  console.log('\n=== Error Handling Example ===');

  try {
    const tutor = sdk.createPhysicalAITutor();
    const response = await tutor.chat(
      'What is quantum computing?', // Outside textbook scope
      null,
      []
    );
    console.log('Answer:', response.answer);
  } catch (error) {
    console.error('Error:', error.message);
  }
}

// Run examples
if (require.main === module) {
  main()
    .then(() => exampleToolUsage())
    .then(() => exampleErrorHandling())
    .catch(console.error);
}

module.exports = {
  main,
  exampleToolUsage,
  exampleErrorHandling
};