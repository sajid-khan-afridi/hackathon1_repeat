import '@testing-library/jest-dom';

// Mock Docusaurus context
jest.mock('@docusaurus/theme-common/internal', () => ({
  DocusaurusContextProvider: ({ children }) => children,
}));

jest.mock('@docusaurus/theme-common', () => ({
  useColorMode: () => ({
    colorMode: 'light',
    setColorMode: jest.fn(),
  }),
}));

jest.mock('@docusaurus/useIsBrowser', () => ({
  default: () => true,
}));