import React from 'react';
import styles from './Translation.module.css';

interface TranslateButtonProps {
  onTranslate: () => void;
  onShowOriginal: () => void;
  currentLanguage: 'english' | 'urdu';
  disabled?: boolean;
  loading?: boolean;
}

export const TranslateButton: React.FC<TranslateButtonProps> = ({
  onTranslate,
  onShowOriginal,
  currentLanguage,
  disabled = false,
  loading = false,
}) => {
  return (
    <div className={styles.translateButtonContainer}>
      {currentLanguage === 'english' ? (
        <button
          className={styles.translateButton}
          onClick={onTranslate}
          disabled={disabled || loading}
          title="Translate this chapter to Urdu"
        >
          {loading ? (
            <>
              <span className={styles.spinner}></span>
              Translating...
            </>
          ) : (
            <>
              <span className={styles.icon}>🌐</span>
              Translate to Urdu
            </>
          )}
        </button>
      ) : (
        <button
          className={styles.originalButton}
          onClick={onShowOriginal}
          disabled={disabled}
          title="Show original English content"
        >
          <span className={styles.icon}>🔙</span>
          Show Original (English)
        </button>
      )}
    </div>
  );
};
