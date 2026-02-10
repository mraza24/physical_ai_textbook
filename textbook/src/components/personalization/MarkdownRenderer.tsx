/**
 * MarkdownRenderer Component
 *
 * Professional markdown rendering with support for:
 * - GitHub Flavored Markdown (GFM) - tables, strikethrough, task lists
 * - Math equations (LaTeX) with KaTeX
 * - Syntax highlighting for code blocks
 * - Preserves original formatting from AI transformations
 *
 * Replaces dangerouslySetInnerHTML approach for safe, React-based rendering.
 */

import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import rehypeHighlight from 'rehype-highlight';
import 'katex/dist/katex.min.css'; // KaTeX CSS for math rendering
import 'highlight.js/styles/github-dark.css'; // Code highlighting theme

interface MarkdownRendererProps {
  /** Markdown content to render (can be original, personalized, or translated) */
  content: string;
  /** Optional CSS class for styling */
  className?: string;
  /** Whether to apply keyword highlighting (for personalized content) */
  highlightKeywords?: boolean;
}

/**
 * MarkdownRenderer Component
 *
 * Renders markdown content with full support for code blocks, math equations,
 * tables, and other GFM features. Ensures content from AI transformations
 * (personalization/translation) displays correctly.
 */
export const MarkdownRenderer: React.FC<MarkdownRendererProps> = ({
  content,
  className = '',
  highlightKeywords = false,
}) => {
  // KEYWORD HIGHLIGHTING: Replace key terms with <mark> tags
  const processedContent = React.useMemo(() => {
    if (!highlightKeywords) return content;

    const keywords = ['Nodes', 'Launch', 'Topics', 'Communication', 'ROS 2', 'Services', 'Actions', 'Publishers', 'Subscribers'];
    let processed = content;

    keywords.forEach(keyword => {
      // Use regex to replace whole words only, preserving markdown
      const regex = new RegExp(`\\b(${keyword})\\b`, 'g');
      processed = processed.replace(regex, (match) => {
        // FORCE YELLOW HIGHLIGHTS: Pure yellow background for demo visibility
        return `<mark style="background-color: yellow; font-weight: bold; padding: 0.1rem 0.3rem; border-radius: 3px;">${match}</mark>`;
      });
    });

    return processed;
  }, [content, highlightKeywords]);

  return (
    <div className={`markdown-renderer ${className}`}>
      <ReactMarkdown
        remarkPlugins={[
          remarkGfm, // GitHub Flavored Markdown (tables, strikethrough, task lists)
          remarkMath, // LaTeX math support
        ]}
        rehypePlugins={[
          rehypeKatex, // Render LaTeX with KaTeX
          rehypeHighlight, // Syntax highlighting for code blocks
        ]}
        components={{
          // Custom component overrides for better styling
          code({ node, inline, className, children, ...props }) {
            const match = /language-(\w+)/.exec(className || '');
            return !inline && match ? (
              <code className={className} {...props}>
                {children}
              </code>
            ) : (
              <code className={className} {...props}>
                {children}
              </code>
            );
          },
          // Ensure tables have proper styling
          table({ children }) {
            return (
              <div className="table-wrapper">
                <table>{children}</table>
              </div>
            );
          },
          // Add custom styling for blockquotes
          blockquote({ children }) {
            return <blockquote className="markdown-blockquote">{children}</blockquote>;
          },
          // Ensure links open in new tab for external URLs
          a({ href, children }) {
            const isExternal = href?.startsWith('http');
            return (
              <a
                href={href}
                target={isExternal ? '_blank' : undefined}
                rel={isExternal ? 'noopener noreferrer' : undefined}
              >
                {children}
              </a>
            );
          },
        }}
      >
        {processedContent}
      </ReactMarkdown>
    </div>
  );
};
