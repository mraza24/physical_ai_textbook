/**
 * Swizzled DocItem/Layout Component
 *
 * This wraps the default Docusaurus doc item layout and injects the ChapterActions component.
 * Based on Docusaurus 3.9.2 theme structure.
 *
 * Swizzle command used: npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChapterActions } from '../../../components/personalization/ChapterActions';
import { MarkdownRenderer } from '../../../components/personalization/MarkdownRenderer';
import { useAuth } from '../../../hooks/useAuth';
import { useContentPersistence } from '../../../hooks/useContentPersistence';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const { getStoredContent, storeContent, clearContent } = useContentPersistence();
  const [transformedContent, setTransformedContent] = useState<string | null>(null);
  const [contentType, setContentType] = useState<'personalized' | 'translated' | 'original'>('original');
  const [urduAutoTrigger, setUrduAutoTrigger] = useState(false);

  // Check if current page is a docs page (works with any baseUrl)
  const isDocsPage = location.pathname.includes('/docs/');

  // Check for ?lang=ur in URL and trigger Urdu translation
  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const lang = params.get('lang');

    if (lang === 'ur' && isAuthenticated) {
      setUrduAutoTrigger(true);
    } else {
      setUrduAutoTrigger(false);
    }
  }, [location.search, isAuthenticated]);

  // Restore persisted content on mount or navigation
  useEffect(() => {
    const stored = getStoredContent(location.pathname);

    console.log('[DocItem/Layout] Restoring persisted content:', {
      pathname: location.pathname,
      hasStored: !!stored,
      storedType: stored?.type,
      contentLength: stored?.content?.length
    });

    if (stored) {
      setTransformedContent(stored.content);
      setContentType(stored.type);
      console.log('[DocItem/Layout] Content restored from localStorage');
    } else {
      setTransformedContent(null);
      setContentType('original');
      console.log('[DocItem/Layout] No stored content - using original');
    }
  }, [location.pathname, getStoredContent]);

  // Extract original markdown content from the document
  // This is a client-side operation, so we use BrowserOnly
  const getOriginalContent = (): string => {
    if (typeof window === 'undefined') return '';

    const markdown = document.querySelector('.markdown');
    if (!markdown) return '';

    // Extract text content as a simplified markdown representation
    // TODO: In production, fetch the actual MDX source file content
    return markdown.textContent || '';
  };

  const handleContentChange = (transformedMarkdown: string, type: 'personalized' | 'translated' | 'original') => {
    console.log('[DocItem/Layout] handleContentChange called:', {
      type,
      contentLength: transformedMarkdown.length,
      contentPreview: transformedMarkdown.substring(0, 100),
      pathname: location.pathname
    });

    if (type === 'original') {
      // Reset to original content and clear storage
      setTransformedContent(null);
      setContentType('original');
      clearContent(location.pathname);
    } else {
      // Store transformed content for rendering with MarkdownRenderer
      setTransformedContent(transformedMarkdown);
      setContentType(type);
      // Persist to localStorage for multi-chapter state
      storeContent(location.pathname, transformedMarkdown, type);

      console.log('[DocItem/Layout] Content state updated:', {
        hasTransformedContent: !!transformedMarkdown,
        contentType: type
      });
    }
  };

  // HARD SWAP: Strict conditional - NO CSS hiding, pure React swap
  const isUrdu = contentType === 'translated';
  const isPersonalized = contentType === 'personalized';

  console.log('[DocItem/Layout] HARD SWAP render:', {
    isUrdu,
    isPersonalized,
    contentType,
    hasContent: !!transformedContent
  });

  return (
    <>
      {/* Buttons */}
      {isDocsPage && (
        <BrowserOnly>
          {() => (
            <ChapterActions
              chapterId={location.pathname}
              originalContent={getOriginalContent()}
              onContentChange={handleContentChange}
              autoTriggerUrdu={urduAutoTrigger}
            />
          )}
        </BrowserOnly>
      )}

      {/* HARD SWAP: Strict ternary - NEVER both */}
      {isUrdu ? (
        <div className='urdu-view' key="urdu" style={{ display: 'block', direction: 'rtl', textAlign: 'right' }}>
          {/* YELLOW AI CONTAINER */}
          <div style={{
            background: '#fef3c7',
            padding: '1.5rem',
            borderRight: '4px solid #f59e0b',
            marginBottom: '2rem',
            borderRadius: '8px',
            textAlign: 'right'
          }}>
            <h2 style={{ margin: '0 0 1rem 0', color: '#92400e', fontWeight: 'bold', fontSize: '1.3rem' }}>
              اردو ترجمہ (AI-Generated)
            </h2>
            <p style={{ margin: 0, fontSize: '0.95rem', color: '#78350f' }}>
              مصنوعی ذہانت سے تیار کردہ
            </p>
          </div>
          <MarkdownRenderer
            content={transformedContent!}
            className="content-translated"
          />
        </div>
      ) : isPersonalized ? (
        <div className='personalized-view' key="personalized" style={{
          border: '6px solid #22c55e',
          backgroundColor: '#f0fdf4',
          padding: '15px',
          borderRadius: '8px'
        }}>
          <MarkdownRenderer
            content={transformedContent!}
            className="content-personalized"
          />
        </div>
      ) : (
        <div className='english-view' key="english">
          <Layout {...props} />
        </div>
      )}
    </>
  );
}
