import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

interface UserProfile {
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
}

function PersonalizedTOCContent() {
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [showRecommendations, setShowRecommendations] = useState(false);

  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Check if we should show personalized content
    const showPersonalized = localStorage.getItem('show_personalized');
    if (showPersonalized === 'true') {
      setShowRecommendations(true);
      localStorage.removeItem('show_personalized');
    }

    // Load user profile from localStorage
    const profileStr = localStorage.getItem('user_profile');
    if (profileStr) {
      try {
        setUserProfile(JSON.parse(profileStr));
      } catch (e) {
        console.error('Failed to parse user profile:', e);
      }
    }
  }, []);

  // Don't render if no profile or not showing recommendations
  if (!userProfile || !showRecommendations) {
    return null;
  }

  // Determine recommended chapters based on user profile
  const getRecommendedModules = (): number[] => {
    const { software_background, hardware_experience } = userProfile;

    if (software_background === 'Beginner') {
      // Beginners should focus on Module 1 (ROS 2) and Module 2 (Simulation)
      return [1, 2];
    } else if (software_background === 'Intermediate') {
      // Intermediate users can handle all modules
      if (hardware_experience === 'None' || hardware_experience === 'Basic') {
        return [1, 2, 3]; // Focus on ROS, Simulation, and Isaac
      } else {
        return [1, 3, 4]; // Can dive into Isaac and VLA
      }
    } else {
      // Expert users should focus on advanced topics
      return [3, 4]; // Isaac and VLA models
    }
  };

  const recommendedModules = getRecommendedModules();

  return (
    <div className={styles.personalizedBanner}>
      <div className={styles.bannerContent}>
        <span className={styles.bannerIcon}>✨</span>
        <div className={styles.bannerText}>
          <strong>Personalized for You!</strong>
          <span className={styles.bannerSubtext}>
            Based on your {userProfile.software_background} software level and{' '}
            {userProfile.hardware_experience} hardware experience
          </span>
        </div>
      </div>
      <div className={styles.recommendationNote}>
        Recommended modules for you are highlighted with{' '}
        <span className={styles.recommendedBadgeInline}>Recommended for You</span> badges below.
      </div>
    </div>
  );
}

export default function PersonalizedTOC() {
  return (
    <BrowserOnly fallback={null}>
      {() => <PersonalizedTOCContent />}
    </BrowserOnly>
  );
}

// Export utility function to check if a module is recommended
export function useRecommendedModules() {
  const [recommendedModules, setRecommendedModules] = useState<number[]>([]);

  useEffect(() => {
    if (typeof window === 'undefined') return;

    const profileStr = localStorage.getItem('user_profile');
    if (!profileStr) return;

    try {
      const userProfile: UserProfile = JSON.parse(profileStr);
      const { software_background, hardware_experience } = userProfile;

      let modules: number[] = [];
      if (software_background === 'Beginner') {
        modules = [1, 2];
      } else if (software_background === 'Intermediate') {
        if (hardware_experience === 'None' || hardware_experience === 'Basic') {
          modules = [1, 2, 3];
        } else {
          modules = [1, 3, 4];
        }
      } else {
        modules = [3, 4];
      }

      setRecommendedModules(modules);
    } catch (e) {
      console.error('Failed to parse user profile:', e);
    }
  }, []);

  return recommendedModules;
}
