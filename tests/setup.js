require('@testing-library/jest-dom');

// Note: Docusaurus mocks are defined in tests/__mocks__/ and mapped in jest.config.js

// Mock scrollIntoView (not available in jsdom)
Element.prototype.scrollIntoView = jest.fn();

// Mock technical terms glossary data
jest.mock('@site/src/data/technical-terms.json', () => ({
  terms: [
    {
      term: 'ROS 2',
      category: 'platform',
      translationStatus: 'preserve',
      contextualExplanation: 'روبوٹ آپریٹنگ سسٹم 2',
    },
    {
      term: 'publisher',
      category: 'ros-concept',
      translationStatus: 'preserve',
      contextualExplanation: 'پبلشر - پیغامات بھیجنے والا نوڈ',
    },
    {
      term: 'subscriber',
      category: 'ros-concept',
      translationStatus: 'preserve',
      contextualExplanation: 'سبسکرائبر - پیغامات وصول کرنے والا نوڈ',
    },
  ],
}), { virtual: true });

// Default mock values for contexts (can be overridden in individual tests)
const mockUserProfile = {
  experienceLevel: 'beginner',
  rosFamiliarity: 'novice',
  hardwareAccess: false,
};

const mockSetUserProfile = jest.fn();

// Create configurable context mocks
let currentUserProfile = mockUserProfile;
let currentLanguage = 'en';

const setMockUserProfile = (profile) => {
  currentUserProfile = profile;
};

const setMockLanguage = (lang) => {
  currentLanguage = lang;
};

const resetMocks = () => {
  currentUserProfile = mockUserProfile;
  currentLanguage = 'en';
  mockSetUserProfile.mockClear();
};

// Export for tests that need to customize mocks
module.exports = { setMockUserProfile, setMockLanguage, resetMocks };

// Mock UserContext
jest.mock('@site/src/context/UserContext', () => ({
  useUserContext: () => ({
    userProfile: currentUserProfile,
    setUserProfile: mockSetUserProfile,
  }),
  UserProvider: ({ children }) => children,
  UserContext: {
    Provider: ({ children }) => children,
  },
}));

// Mock LanguageContext
jest.mock('@site/src/context/LanguageContext', () => ({
  useLanguageContext: () => ({
    language: currentLanguage,
    setLanguage: jest.fn(),
  }),
  LanguageProvider: ({ children }) => children,
  LanguageContext: {
    Provider: ({ children }) => children,
  },
}));

// Reset mocks before each test
beforeEach(() => {
  resetMocks();
});