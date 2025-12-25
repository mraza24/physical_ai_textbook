import React from 'react';
// @site alias docusaurus mein project root ko point karta hai
import RAGChatbot from '@site/src/components/RAGChatbot'; 

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <React.Fragment>
      {/* Ye aapki asli textbook ka content render karega */}
      {children} 
      
      {/* Ye chatbot ko har page ke upar layer karega */}
      <div style={{ position: 'relative', zIndex: 1000 }}>
        <RAGChatbot />
      </div>
    </React.Fragment>
  );
}