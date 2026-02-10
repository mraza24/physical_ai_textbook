import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

interface ModuleBadgeProps {
  moduleNumber: number;
}

interface UserProfile {
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
}

function ModuleBadgeContent({ moduleNumber }: ModuleBadgeProps) {
  const [isRecommended, setIsRecommended] = useState(false);

  useEffect(() => {
    if (typeof window === 'undefined') return;

    const profileStr = localStorage.getItem('user_profile');
    if (!profileStr) return;

    try {
      const userProfile: UserProfile = JSON.parse(profileStr);
      const { software_background, hardware_experience } = userProfile;

      let recommendedModules: number[] = [];
      if (software_background === 'Beginner') {
        recommendedModules = [1, 2];
      } else if (software_background === 'Intermediate') {
        if (hardware_experience === 'None' || hardware_experience === 'Basic') {
          recommendedModules = [1, 2, 3];
        } else {
          recommendedModules = [1, 3, 4];
        }
      } else {
        recommendedModules = [3, 4];
      }

      setIsRecommended(recommendedModules.includes(moduleNumber));
    } catch (e) {
      console.error('Failed to parse user profile:', e);
    }
  }, [moduleNumber]);

  if (!isRecommended) {
    return null;
  }

  return (
    <span className={styles.recommendedBadge}>
      <span className={styles.badgeIcon}>⭐</span>
      Recommended for You
    </span>
  );
}

export default function ModuleBadge({ moduleNumber }: ModuleBadgeProps) {
  return (
    <BrowserOnly fallback={null}>
      {() => <ModuleBadgeContent moduleNumber={moduleNumber} />}
    </BrowserOnly>
  );
}
