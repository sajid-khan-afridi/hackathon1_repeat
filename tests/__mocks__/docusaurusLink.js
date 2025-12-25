/**
 * Mock for @docusaurus/Link
 * Returns a simple anchor element for testing
 */
import React from 'react';

function Link({ children, to, href, ...props }) {
  return React.createElement('a', { href: to || href, ...props }, children);
}

export default Link;
