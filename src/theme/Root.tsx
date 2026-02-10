import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {/* Ye children aapki poori website ka content hai */}
      {children}
      
      {/* Ye naya Chatbot hai jo har page par render hoga */}
      <Chatbot />
    </>
  );
}