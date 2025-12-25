import { useState } from 'react';
import { useAuth } from './useAuth';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';

export interface PersonalizationResult {
  personalizedContent: string;
  cacheHit: boolean;
  profile: any;
}

export const usePersonalization = () => {
  const { isAuthenticated } = useAuth();
  const [personalizing, setPersonalizing] = useState(false);
  const [personalized, setPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);

  /**
   * Personalize chapter content
   */
  const personalizeChapter = async (chapterId: string, originalContent: string) => {
    if (!isAuthenticated) {
      setError('Please sign in to personalize content');
      return;
    }

    setPersonalizing(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        throw new Error('Not authenticated. Please sign in.');
      }

      const response = await fetch(`${API_BASE_URL}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({ chapterId, originalContent }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.error?.message || 'Personalization failed');
      }

      const data: PersonalizationResult = await response.json();

      setPersonalizedContent(data.personalizedContent);
      setPersonalized(true);
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Personalization failed';
      setError(message);
      setPersonalized(false);
    } finally {
      setPersonalizing(false);
    }
  };

  /**
   * Reset to original content
   */
  const resetChapter = () => {
    setPersonalized(false);
    setPersonalizedContent(null);
    setError(null);
  };

  return {
    personalizing,
    personalized,
    personalizedContent,
    error,
    personalizeChapter,
    resetChapter,
  };
};
