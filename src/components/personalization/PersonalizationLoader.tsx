import React, { useEffect, useState } from 'react';
import styles from './Personalization.module.css';

interface PersonalizationLoaderProps {
  timeout?: number; // Timeout in milliseconds
  onTimeout?: () => void;
}

export const PersonalizationLoader: React.FC<PersonalizationLoaderProps> = ({
  timeout = 10000,
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
          : 'Personalizing content based on your profile...'}
      </p>
      <p className={styles.loaderSubtext}>
        Adapting Python/ROS 2 concepts, hardware recommendations, and examples
      </p>
    </div>
  );
};
