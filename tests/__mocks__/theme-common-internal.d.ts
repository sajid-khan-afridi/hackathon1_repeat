import { ReactNode } from 'react';

export interface DocusaurusContextProviderProps {
  children: ReactNode;
  value?: any;
}

export const DocusaurusContextProvider: React.FC<DocusaurusContextProviderProps>;
