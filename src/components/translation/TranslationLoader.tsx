import React, { useEffect, useState } from 'react';
import styles from './Translation.module.css';

interface TranslationLoaderProps {
  timeout?: number; // Timeout in milliseconds
  onTimeout?: () => void;
}

export const TranslationLoader: React.FC<TranslationLoaderProps> = ({
  timeout = 15000,
  onTimeout,
}) => {
  const [timeoutReached, setTimeoutReached] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => {
      setTimeoutReached(true);
      if (onTimeout) {
        onTimeout();
      }
    }, timeout);

    return () => clearTimeout(timer);
  }, [timeout, onTimeout]);

  return (
    <div className={styles.loaderContainer}>
      <div className={styles.spinner}></div>
      <p className={styles.loaderMessage}>
        {timeoutReached
          ? 'This is taking longer than expected. Please wait...'
          : 'Translating to Urdu...'}
      </p>
      <p className={styles.loaderSubtext}>
        Preserving code blocks, transliterating technical terms, maintaining formatting
      </p>
    </div>
  );
};
