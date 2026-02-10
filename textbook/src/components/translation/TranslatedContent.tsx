import React from 'react';
import ReactMarkdown from 'react-markdown'; // 👈 Yeh line ab kaam karegi
import styles from './Translation.module.css';

export const TranslatedContent = ({ content, onShowOriginal }) => {
  if (!content) return null;

  return (
    <div className={styles.translatedContainer}>
      <div className={styles.translatedBanner}>
        <span style={{ fontWeight: 'bold', color: '#1e40af' }}>🌐 (Beta) اردو ترجمہ</span>
        <button onClick={onShowOriginal} className={styles.showOriginalButton}>
          🔙 English
        </button>
      </div>

      <div className={styles.urduBody}>
        {/* 🎯 ReactMarkdown aapke text ko asli headings aur code blocks mein badal dega */}
        <ReactMarkdown>{content}</ReactMarkdown>
      </div>
    </div>
  );
};