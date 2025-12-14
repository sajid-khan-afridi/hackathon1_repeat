module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  setupFilesAfterEnv: ['<rootDir>/tests/setup.js'],
  globals: {
    'ts-jest': {
      tsconfig: {
        jsx: 'react-jsx',
        esModuleInterop: true,
        allowSyntheticDefaultImports: true,
        noUnusedLocals: false,
        noUnusedParameters: false,
        resolveJsonModule: true,
        baseUrl: '.',
        paths: {
          '@site/*': ['./*'],
          '@docusaurus/theme-common/internal': ['./tests/__mocks__/theme-common-internal'],
        },
        typeRoots: ['./node_modules/@types', './tests/__mocks__'],
      },
    },
  },
  moduleNameMapper: {
    '\\.(css|less|scss|sass|md)$': 'identity-obj-proxy',
    '^@/(.*)$': '<rootDir>/src/$1',
    '^@site/(.*)$': '<rootDir>/$1',
    '^@docusaurus/useIsBrowser$': '<rootDir>/tests/__mocks__/useIsBrowser.js',
    '^@docusaurus/theme-common$': '<rootDir>/tests/__mocks__/theme-common.js',
    '^@docusaurus/theme-common/internal$': '<rootDir>/tests/__mocks__/theme-common-internal.js',
  },
  testMatch: [
    '<rootDir>/tests/unit/**/*.test.{js,jsx,ts,tsx}',
    '<rootDir>/src/**/*.test.{js,jsx,ts,tsx}',
  ],
  collectCoverageFrom: [
    'src/**/*.{js,jsx,ts,tsx}',
    '!src/**/*.d.ts',
    '!src/**/*.stories.{js,jsx,ts,tsx}',
  ],
  coverageReporters: ['text', 'lcov', 'html'],
  coverageDirectory: 'coverage',
  coverageThreshold: {
    global: {
      branches: 80,
      functions: 80,
      lines: 80,
      statements: 80,
    },
  },
};