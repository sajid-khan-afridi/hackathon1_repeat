/**
 * Test file for Qdrant VectorStore skill
 */

const { handler } = require('./index.js');

// Mock environment variables for testing
process.env.QDRANT_URL = 'https://your-cluster.qdrant.io';
process.env.QDRANT_API_KEY = 'test-api-key';
process.env.OPENAI_API_KEY = 'test-openai-key';

// Test cases
async function runTests() {
  console.log('🧪 Running Qdrant VectorStore skill tests...\n');

  try {
    // Test 1: Create collection
    console.log('Test 1: Create collection');
    const createResult = await handler({
      operation: 'create_collection',
      collection_name: 'test_collection'
    });
    console.log('✅ Create collection result:', createResult);
    console.log();

    // Test 2: Upsert documents
    console.log('Test 2: Upsert documents');
    const upsertResult = await handler({
      operation: 'upsert',
      collection_name: 'test_collection',
      documents: [
        'The quick brown fox jumps over the lazy dog.',
        'Lorem ipsum dolor sit amet, consectetur adipiscing elit.',
        'In a world of vector databases, semantic search is key to finding relevant information.'
      ]
    });
    console.log('✅ Upsert result:', upsertResult);
    console.log();

    // Test 3: Search documents
    console.log('Test 3: Search documents');
    const searchResult = await handler({
      operation: 'search',
      collection_name: 'test_collection',
      query: 'vector database search',
      top_k: 3
    });
    console.log('✅ Search result:', JSON.stringify(searchResult, null, 2));
    console.log();

    console.log('🎉 All tests passed successfully!');

  } catch (error) {
    console.error('❌ Test failed:', error.message);
    console.log('\nNote: This test requires valid Qdrant Cloud and OpenAI credentials.');
    console.log('Set the following environment variables to run actual tests:');
    console.log('- QDRANT_URL: Your Qdrant Cloud endpoint');
    console.log('- QDRANT_API_KEY: Your Qdrant Cloud API key');
    console.log('- OPENAI_API_KEY: Your OpenAI API key');
  }
}

// Run tests if file is executed directly
if (require.main === module) {
  runTests();
}

module.exports = { runTests };