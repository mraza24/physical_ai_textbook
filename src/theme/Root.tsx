/**
 * Root Component
 * Wraps the entire Docusaurus site to include global components
 */

import React from 'react';
import Chatbot from '../components/Chatbot';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
