import React from 'react';
import styles from './Personalization.module.css';

interface PersonalizedContentProps {
  content: string;
  onReset: () => void;
}

export const PersonalizedContent: React.FC<PersonalizedContentProps> = ({
  content,
  onReset,
}) => {
  return (
    <div className={styles.personalizedContentContainer}>
      <div className={styles.personalizedBanner}>
        <span className={styles.icon}>✨</span>
        <span>
          This content has been personalized for your experience level and hardware
        </span>
        <button className={styles.resetButton} onClick={onReset} title="Show original content">
          Reset to Default
        </button>
      </div>

      <div
        className={styles.personalizedContent}
        dangerouslySetInnerHTML={{ __html: content }}
      />
    </div>
  );
};
