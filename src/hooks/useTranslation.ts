import { useState, useEffect } from 'react';
import { useAuth } from './useAuth';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';

export interface TranslationResult {
  translatedContent: string;
  cacheHit: boolean;
  targetLanguage: string;
  technicalTerms: Record<string, string>;
}

export const useTranslation = () => {
  const { isAuthenticated, profile, updateProfile } = useAuth();
  const [translating, setTranslating] = useState(false);
  const [language, setLanguage] = useState<'english' | 'urdu'>('english');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [technicalTerms, setTechnicalTerms] = useState<Record<string, string>>({});
  const [error, setError] = useState<string | null>(null);

  // Load language preference on mount (if authenticated)
  useEffect(() => {
    if (isAuthenticated && profile) {
      if (profile.language_preference === 'Urdu') {
        setLanguage('urdu');
      }
    }
  }, [isAuthenticated, profile]);

  /**
   * Translate chapter to Urdu
   */
  const translateToUrdu = async (chapterId: string, originalContent: string) => {
    setTranslating(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapterId,
          originalContent,
          targetLanguage: 'urdu',
        }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error?.message || 'Translation failed');
      }

      const data: TranslationResult = await response.json();

      setTranslatedContent(data.translatedContent);
      setTechnicalTerms(data.technicalTerms);
      setLanguage('urdu');

      // Persist language preference if authenticated
      if (isAuthenticated) {
        try {
          await updateProfile({ language_preference: 'Urdu' });
        } catch (err) {
          console.error('Failed to persist language preference:', err);
        }
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Translation failed';
      setError(message);
    } finally {
      setTranslating(false);
    }
  };

  /**
   * Show original English content
   */
  const showOriginal = async () => {
    setLanguage('english');
    setTranslatedContent(null);
    setTechnicalTerms({});
    setError(null);

    // Persist language preference if authenticated
    if (isAuthenticated) {
      try {
        await updateProfile({ language_preference: 'English' });
      } catch (err) {
        console.error('Failed to persist language preference:', err);
      }
    }
  };

  return {
    translating,
    language,
    translatedContent,
    technicalTerms,
    error,
    translateToUrdu,
    showOriginal,
  };
};
