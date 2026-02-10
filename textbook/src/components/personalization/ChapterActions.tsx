import React, { useState, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../../hooks/useAuth';
import { usePersonalization } from '../../hooks/usePersonalization';
import { useTranslation } from '../../hooks/useTranslation';
import styles from './ChapterActions.module.css';
import { URDU_TRANSLATIONS } from '../../data/urduTranslations';

interface ChapterActionsProps {
  chapterId: string;
  originalContent: string;
  onContentChange: (newContent: string, type: 'personalized' | 'translated' | 'original') => void;
  autoTriggerUrdu?: boolean;
}

export const ChapterActions: React.FC<ChapterActionsProps> = ({
  chapterId,
  originalContent,
  onContentChange,
  autoTriggerUrdu = false,
}) => {
  const history = useHistory();
  const { user, profile } = useAuth();
  
  const {
    personalized,
    personalizedContent,
    resetChapter: resetPersonalization,
  } = usePersonalization();

  const [showUrdu, setShowUrdu] = useState(false);
  const [urduTranslations, setUrduTranslations] = useState<Record<string, string>>({});
  const isPersonalizing = useRef(false);
  const isTranslating = useRef(false);

 // ChapterActions.tsx mein Content Switching Logic ko aise update karein:
useEffect(() => {
  if (personalized && personalizedContent) {
    onContentChange(personalizedContent, 'personalized');
  } else if (showUrdu) {
    // 1. Current URL se slug nikaalein (e.g., chapter1-1-ros2-fundamentals)
    const slug = chapterId.split('/').pop() || '';

    // 2. Data dhoondne ka behtareen tareeka (Search in all keys)
    const translationKey = Object.keys(URDU_TRANSLATIONS).find(key => 
      key.includes(slug) // Agar key mein 'chapter1-1-ros2-fundamentals' kahin bhi hai
    );

    const urduText = translationKey 
      ? URDU_TRANSLATIONS[translationKey] 
      : "معذرت، اس باب کا ترجمہ دستیاب نہیں ہے۔";

    // 3. Content bhejein (Heading ke saath)
const finalContent = urduText;    
    onContentChange(finalContent, 'translated');
  } else {
    onContentChange(originalContent, 'original');
  }
}, [personalized, personalizedContent, showUrdu, originalContent, chapterId]);
  // ==========================================
  // 🎯 PERSONALIZE LOGIC (Beginner/Expert Fix)
  // ==========================================
  const handlePersonalize = () => {
    const isLoggedIn = localStorage.getItem('isLoggedIn') === 'true';
    
    if (!isLoggedIn) {
      window.location.assign('/physical_ai_textbook/signup');
      return;
    }

    if (isPersonalizing.current) return;
    isPersonalizing.current = true;
    setShowUrdu(false);

    // Profile se level lein, warna "Beginner" fallback
    const userRole = (profile as any)?.software_background || (user as any)?.software_background || 'Beginner';
    
    window.dispatchEvent(new CustomEvent('bulldog:notify', {
      detail: {
        message: `✨ As a ${userRole}, I have personalized this for you!`,
        type: 'personalization'
      }
    }));

    const personalizedText = `# Personalized for ${userRole} Level\n\n${originalContent}\n\n> 💡 **AI Insight**: Content tailored for your ${userRole} background.`;
    onContentChange(personalizedText, 'personalized');

    setTimeout(() => { isPersonalizing.current = false; }, 1000);
  };

  // ==========================================
  // 🌐 TRANSLATE LOGIC
  // ==========================================
  const handleTranslate = () => {
    const isLoggedIn = localStorage.getItem('isLoggedIn') === 'true';
    
    if (!isLoggedIn) {
      window.location.assign('/physical_ai_textbook/signup');
      return;
    }

    if (isTranslating.current) return;
    isTranslating.current = true;

    if (!showUrdu) {
      resetPersonalization();
      setShowUrdu(true);
      window.dispatchEvent(new CustomEvent('bulldog:notify', {
        detail: { message: '✅ Urdu Translation Active', type: 'translation' }
      }));
    } else {
      setShowUrdu(false);
    }

    setTimeout(() => { isTranslating.current = false; }, 500);
  };

  const sessionActive = typeof window !== 'undefined' && localStorage.getItem('isLoggedIn') === 'true';

  return (
    <div className={styles.chapterActionsContainer}>
      <div className={styles.actionsBar}>
        <button
          className={`${styles.actionButton} ${styles.personalizeButton} ${!sessionActive ? styles.lockedButton : ''}`}
          onClick={handlePersonalize}
        >
          <span>✨ {personalized ? 'Personalized' : 'Personalize'}</span>
          {!sessionActive && <span className={styles.lockIcon}> 🔒</span>}
        </button>

        <button
          className={`${styles.actionButton} ${styles.translateButton} ${!sessionActive ? styles.lockedButton : ''}`}
          onClick={handleTranslate}
        >
          <span>{showUrdu ? '🔙 English' : '🌐 Urdu'}</span>
          {!sessionActive && <span className={styles.lockIcon}> 🔒</span>}
        </button>
      </div>
    </div>
  );
};