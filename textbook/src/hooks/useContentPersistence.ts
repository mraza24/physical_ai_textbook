/**
 * useContentPersistence Hook
 *
 * Provides multi-chapter content persistence using localStorage.
 * Remembers user's personalization/translation state across chapters.
 *
 * Features:
 * - Stores transformed content per chapter
 * - Persists transformation type (personalized/translated)
 * - Auto-expires after 24 hours
 * - Clears on logout
 */

import { useState, useEffect } from 'react';

interface ContentState {
  content: string;
  type: 'personalized' | 'translated' | 'original';
  timestamp: number;
  cacheKey?: string;
}

interface UseContentPersistenceReturn {
  /** Get stored content for a chapter */
  getStoredContent: (chapterPath: string) => ContentState | null;
  /** Store transformed content for a chapter */
  storeContent: (chapterPath: string, content: string, type: 'personalized' | 'translated') => void;
  /** Clear stored content for a chapter */
  clearContent: (chapterPath: string) => void;
  /** Clear all stored content */
  clearAllContent: () => void;
  /** Check if content is stored for a chapter */
  hasStoredContent: (chapterPath: string) => boolean;
}

const STORAGE_KEY_PREFIX = 'chapter_content_';
const EXPIRY_MS = 24 * 60 * 60 * 1000; // 24 hours

/**
 * useContentPersistence Hook
 *
 * Manages localStorage persistence for transformed chapter content.
 * Automatically expires stale content and provides utilities for
 * storing/retrieving content across page navigations.
 */
export const useContentPersistence = (): UseContentPersistenceReturn => {
  const [, setForceUpdate] = useState(0);

  // Clean up expired content on mount
  useEffect(() => {
    cleanupExpiredContent();
  }, []);

  /**
   * Generate storage key for a chapter
   */
  const getStorageKey = (chapterPath: string): string => {
    return `${STORAGE_KEY_PREFIX}${chapterPath}`;
  };

  /**
   * Clean up expired content from localStorage
   */
  const cleanupExpiredContent = () => {
    if (typeof window === 'undefined') return;

    const now = Date.now();
    const keys = Object.keys(localStorage);

    keys.forEach((key) => {
      if (key.startsWith(STORAGE_KEY_PREFIX)) {
        try {
          const stored = localStorage.getItem(key);
          if (stored) {
            const state: ContentState = JSON.parse(stored);
            if (now - state.timestamp > EXPIRY_MS) {
              localStorage.removeItem(key);
            }
          }
        } catch (error) {
          // Invalid JSON, remove it
          localStorage.removeItem(key);
        }
      }
    });
  };

  /**
   * Get stored content for a chapter
   */
  const getStoredContent = (chapterPath: string): ContentState | null => {
    if (typeof window === 'undefined') return null;

    try {
      const key = getStorageKey(chapterPath);
      const stored = localStorage.getItem(key);

      if (!stored) return null;

      const state: ContentState = JSON.parse(stored);

      // Check if expired
      if (Date.now() - state.timestamp > EXPIRY_MS) {
        localStorage.removeItem(key);
        return null;
      }

      return state;
    } catch (error) {
      console.error('Failed to retrieve stored content:', error);
      return null;
    }
  };

  /**
   * Store transformed content for a chapter
   */
  const storeContent = (
    chapterPath: string,
    content: string,
    type: 'personalized' | 'translated'
  ) => {
    if (typeof window === 'undefined') return;

    try {
      const key = getStorageKey(chapterPath);
      const state: ContentState = {
        content,
        type,
        timestamp: Date.now(),
      };

      localStorage.setItem(key, JSON.stringify(state));
      setForceUpdate((prev) => prev + 1); // Trigger re-render
    } catch (error) {
      console.error('Failed to store content:', error);
    }
  };

  /**
   * Clear stored content for a chapter
   */
  const clearContent = (chapterPath: string) => {
    if (typeof window === 'undefined') return;

    try {
      const key = getStorageKey(chapterPath);
      localStorage.removeItem(key);
      setForceUpdate((prev) => prev + 1); // Trigger re-render
    } catch (error) {
      console.error('Failed to clear content:', error);
    }
  };

  /**
   * Clear all stored content
   */
  const clearAllContent = () => {
    if (typeof window === 'undefined') return;

    try {
      const keys = Object.keys(localStorage);
      keys.forEach((key) => {
        if (key.startsWith(STORAGE_KEY_PREFIX)) {
          localStorage.removeItem(key);
        }
      });
      setForceUpdate((prev) => prev + 1); // Trigger re-render
    } catch (error) {
      console.error('Failed to clear all content:', error);
    }
  };

  /**
   * Check if content is stored for a chapter
   */
  const hasStoredContent = (chapterPath: string): boolean => {
    return getStoredContent(chapterPath) !== null;
  };

  return {
    getStoredContent,
    storeContent,
    clearContent,
    clearAllContent,
    hasStoredContent,
  };
};
