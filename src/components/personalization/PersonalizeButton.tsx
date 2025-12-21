import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './Personalization.module.css';

interface PersonalizeButtonProps {
  onClick: () => void;
  disabled?: boolean;
  loading?: boolean;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  onClick,
  disabled = false,
  loading = false,
}) => {
  const { isAuthenticated } = useAuth();

  const handleClick = () => {
    if (!isAuthenticated) {
      alert('Please sign in to personalize content');
      return;
    }

    onClick();
  };

  return (
    <div className={styles.personalizeButtonContainer}>
      <button
        className={styles.personalizeButton}
        onClick={handleClick}
        disabled={disabled || loading}
        title={
          !isAuthenticated
            ? 'Sign in to personalize content based on your background'
            : 'Adapt this chapter to your experience level and hardware'
        }
      >
        {loading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : (
          <>
            <span className={styles.icon}>✨</span>
            Personalize This Chapter
          </>
        )}
      </button>

      {!isAuthenticated && (
        <p className={styles.tooltip}>
          Sign in to personalize content based on your Python/ROS 2 experience and hardware
        </p>
      )}
    </div>
  );
};
