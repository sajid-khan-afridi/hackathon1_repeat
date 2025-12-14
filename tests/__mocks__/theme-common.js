// Mock for @docusaurus/theme-common
module.exports = {
  useColorMode: () => ({
    colorMode: 'light',
    setColorMode: jest.fn(),
  }),
};
