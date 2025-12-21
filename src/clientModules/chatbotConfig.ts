/**
 * Client-side configuration for ChatbotWidget
 * Sets the production API URL for the RAG backend
 */

// Set the chatbot API URL globally
if (typeof window !== 'undefined') {
  (window as any).CHATBOT_API_URL = (typeof process !== 'undefined' && process.env?.NODE_ENV === 'production')
    ? 'https://hackathon1repeat-production.up.railway.app'
    : 'http://localhost:8000';

  console.log('[Chatbot] API URL configured:', (window as any).CHATBOT_API_URL);
}

export {};
