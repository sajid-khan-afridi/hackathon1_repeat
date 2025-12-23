/**
 * Client-side configuration for ChatbotWidget
 * Sets the production API URL for the RAG backend
 */

// Get API URL from Docusaurus config or fallback
const getApiUrl = (): string => {
  if (typeof window !== 'undefined') {
    // Check for Docusaurus config (primary source)
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
      return docusaurusConfig.siteConfig.customFields.apiUrl;
    }

    // Fallback: Check if we're on the production domain
    if (window.location.hostname === 'sajid-khan-afridi.github.io') {
      return 'https://hackathon1repeat-production.up.railway.app';
    }
  }
  // Default to localhost for development
  return 'http://localhost:8000';
};

// Set the chatbot API URL globally
if (typeof window !== 'undefined') {
  (window as any).CHATBOT_API_URL = getApiUrl();
  console.log('[Chatbot] API URL configured:', (window as any).CHATBOT_API_URL);
}

export {};
