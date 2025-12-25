declare module '@docusaurus/Link' {
  import { ComponentType, ReactNode, AnchorHTMLAttributes } from 'react';

  interface LinkProps extends AnchorHTMLAttributes<HTMLAnchorElement> {
    to?: string;
    href?: string;
    children?: ReactNode;
  }

  const Link: ComponentType<LinkProps>;
  export default Link;
}
