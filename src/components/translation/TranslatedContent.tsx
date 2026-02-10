import React from 'react';
import styles from '../Translation.module.css';

interface TranslatedContentProps {
  content: string;
  onShowOriginal: () => void;
}

export const TranslatedContent: React.FC<TranslatedContentProps> = ({
  content,
  onShowOriginal,
}) => {
  if (!content) return <div style={{padding: '20px', textAlign: 'center'}}>لوڈ ہو رہا ہے...</div>;

  // 🚀 Formatting line breaks and cleaning extra quotes
  const finalHtml = content
    .trim()
    .replace(/\\n/g, '<br/>')
    .replace(/\n/g, '<br/>');

  return (
    <div className={styles.translatedContainer} dir="rtl">
      {/* Banner */}
      <div className={styles.translatedBanner} dir="ltr" style={{
        display: 'flex', justifyContent: 'space-between', alignItems: 'center',
        padding: '10px 20px', background: '#f8fafc', borderBottom: '2px solid #e2e8f0'
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <span style={{ fontSize: '1.2rem' }}>🌐</span>
          <span style={{ fontWeight: 'bold', color: '#1e40af' }}>Urdu Mode Active</span>
        </div>
        <button onClick={onShowOriginal} style={{
          padding: '6px 15px', borderRadius: '5px', background: '#ef4444', color: 'white', border: 'none', cursor: 'pointer', fontWeight: 'bold'
        }}>
          Back to English
        </button>
      </div>

      {/* 🎯 THE FIX: Force HTML Rendering */}
      <div 
        className={styles.urduBody} 
        style={{ 
          textAlign: 'right', padding: '30px', direction: 'rtl',
          fontSize: '1.4rem', lineHeight: '2.3',
          fontFamily: '"Noto Nastaliq Urdu", "Urdu Typesetting", serif',
          color: '#1e293b'
        }}
        dangerouslySetInnerHTML={{ __html: finalHtml }}
      />
    </div>
  );
};