import { useState } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface PersonalizationResult {
  transformed_content: string;
  metadata: {
    model: string;
    changes_made: number;
    complexity_level: string;
    preserved_terms: string[];
    cached: boolean;
  };
  cache_key: string;
  processing_time_ms: number;
  validation_warnings?: string[];
}

export const usePersonalization = () => {
  // ✅ SAFE: Get API URL from Docusaurus context (no process.env in browser!)
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const { isAuthenticated } = useAuth();
  const [personalizing, setPersonalizing] = useState(false);
  const [personalized, setPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);

  /**
   * Personalize chapter content
   */
  const personalizeChapter = async (chapterPath: string, content: string) => {
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
        body: JSON.stringify({ chapterPath, content }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Personalization failed');
      }

      const data: PersonalizationResult = await response.json();

      setPersonalizedContent(data.transformed_content);
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
