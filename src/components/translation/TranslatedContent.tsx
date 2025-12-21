import React, { useState } from 'react';
import styles from './Translation.module.css';

interface TranslatedContentProps {
  content: string;
  technicalTerms: Record<string, string>;
  onShowOriginal: () => void;
}

export const TranslatedContent: React.FC<TranslatedContentProps> = ({
  content,
  technicalTerms,
  onShowOriginal,
}) => {
  const [showGlossary, setShowGlossary] = useState(false);

  return (
    <div className={styles.translatedContainer}>
      <div className={styles.translatedBanner}>
        <span className={styles.icon}>🌐</span>
        <span>This content has been translated to Urdu (اردو)</span>
        <button
          className={styles.showOriginalButton}
          onClick={onShowOriginal}
          title="Show original English content"
        >
          Show Original
        </button>
      </div>

      {Object.keys(technicalTerms).length > 0 && (
        <div className={styles.technicalTermsContainer}>
          <button
            className={styles.glossaryToggle}
            onClick={() => setShowGlossary(!showGlossary)}
          >
            {showGlossary ? '▼' : '▶'} Technical Terms Glossary ({Object.keys(technicalTerms).length})
          </button>

          {showGlossary && (
            <div className={styles.glossary}>
              {Object.entries(technicalTerms).map(([english, urdu]) => (
                <div key={english} className={styles.glossaryItem}>
                  <span className={styles.englishTerm}>{english}</span>
                  <span className={styles.arrow}>→</span>
                  <span className={styles.urduTerm}>{urdu}</span>
                </div>
              ))}
            </div>
          )}
        </div>
      )}

      <div
        className={`${styles.translatedContent} ${styles.rtl}`}
        dir="rtl"
        lang="ur"
        dangerouslySetInnerHTML={{ __html: content }}
      />
    </div>
  );
};
