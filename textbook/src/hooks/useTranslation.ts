import { useState, useEffect } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface TranslationResult {
  translated_content: string;
  metadata: {
    model: string;
    preserved_terms: string[];
    target_language: string;
    cached: boolean;
  };
  cache_key: string;
  processing_time_ms: number;
  validation_warnings?: string[];
}

export const useTranslation = () => {
  // ✅ SAFE: Get API URL from Docusaurus context (no process.env in browser!)
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const { isAuthenticated, profile, updateProfile } = useAuth();
  const [translating, setTranslating] = useState(false);
  const [language, setLanguage] = useState<'english' | 'urdu'>('english');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [technicalTerms, setTechnicalTerms] = useState<string[]>([]);
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
  const translateToUrdu = async (chapterPath: string, content: string) => {
    setTranslating(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        throw new Error('Not authenticated. Please sign in.');
      }

      const response = await fetch(`${API_BASE_URL}/api/translate/urdu`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapterPath,
          content,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Translation failed');
      }

      const data: TranslationResult = await response.json();

      setTranslatedContent(data.translated_content);
      setTechnicalTerms(data.metadata.preserved_terms);
      setLanguage('urdu');

      // Persist language preference if authenticated
      if (isAuthenticated && updateProfile) {
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
    setTechnicalTerms([]);
    setError(null);

    // Persist language preference if authenticated
    if (isAuthenticated && updateProfile) {
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
