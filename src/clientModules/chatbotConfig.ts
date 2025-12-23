/**
 * Client-side configuration for ChatbotWidget
 * Sets the production API URL for the RAG backend
 */

// Production API URL
const PRODUCTION_API_URL = 'https://hackathon1repeat-production.up.railway.app';

// Get API URL from Docusaurus config or fallback
const getApiUrl = (): string => {
  if (typeof window !== 'undefined') {
    // Check if running on production domain (GitHub Pages) - highest priority
    if (window.location.hostname === 'sajid-khan-afridi.github.io') {
      return PRODUCTION_API_URL;
    }

    // Check for Docusaurus config
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
      return docusaurusConfig.siteConfig.customFields.apiUrl;
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
