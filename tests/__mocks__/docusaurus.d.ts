// TypeScript declarations for Docusaurus mocks
declare module '@docusaurus/useIsBrowser' {
  const useIsBrowser: () => boolean;
  export default useIsBrowser;
}

declare module '@docusaurus/theme-common' {
  export const useColorMode: () => {
    colorMode: 'light' | 'dark';
    setColorMode: (mode: 'light' | 'dark') => void;
  };
}

declare module '*.module.css' {
  const classes: { [key: string]: string };
  export default classes;
}

declare module '*.css' {
  const classes: { [key: string]: string };
  export default classes;
}