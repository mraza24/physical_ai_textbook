/**
 * Root Component
 * Wraps the entire Docusaurus site to include global components
 * SSG-Safe: Chatbot is lazy-loaded and BrowserOnly-wrapped
 */

import React from 'react';

// Lazy load the chatbot to avoid SSR issues
const RAGChatbot = React.lazy(() => import('../components/RAGChatbot/index'));

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      {/* Suspense fallback ensures no SSR issues */}
      <React.Suspense fallback={<div />}>
        <RAGChatbot />
      </React.Suspense>
    </>
  );
}
