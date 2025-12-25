/**
 * RAG Chatbot Component
 * Floating chatbot with textbook Q&A capabilities
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

// ⚠️ IMPORTANT: Update this URL to your deployed backend
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

interface Citation {
  section_title: string;
  deep_link_url: string;
  chunk_count: number;
}

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  confidence?: number;
  timestamp: Date;
}

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    const query = inputValue.trim();
    if (!query || isLoading) return;

    // Add user message
    const userMessage: ChatMessage = {
      role: 'user',
      content: query,
      timestamp: new Date(),
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: query,
          selected_text: null,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations || [],
        confidence: data.confidence,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, assistantMessage]);

    } catch (err) {
      console.error('Chatbot error:', err);
      setError(err instanceof Error ? err.message : 'Failed to get response');

      // Add error message
      const errorMessage: ChatMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again or check if the backend is running.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClear = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor">
            <path d="M18 6L6 18M6 6l12 12" strokeWidth="2" strokeLinecap="round" />
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>🤖 Ask the Textbook</h3>
            <div className={styles.headerActions}>
              {messages.length > 0 && (
                <button
                  onClick={handleClear}
                  className={styles.clearButton}
                  title="Clear chat"
                >
                  🗑️
                </button>
              )}
              <button
                onClick={() => setIsOpen(false)}
                className={styles.closeButton}
                aria-label="Close chatbot"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.emptyState}>
                <p>👋 Hi! I'm your Physical AI textbook assistant.</p>
                <p>Ask me anything about robotics, AI, or simulation!</p>
                <div className={styles.exampleQuestions}>
                  <p><strong>Try asking:</strong></p>
                  <button onClick={() => setInputValue('What is forward kinematics?')}>
                    What is forward kinematics?
                  </button>
                  <button onClick={() => setInputValue('Explain inverse kinematics')}>
                    Explain inverse kinematics
                  </button>
                  <button onClick={() => setInputValue('What is ROS?')}>
                    What is ROS?
                  </button>
                </div>
              </div>
            ) : (
              messages.map((msg, idx) => (
                <div
                  key={idx}
                  className={`${styles.message} ${styles[msg.role]}`}
                >
                  <div className={styles.messageContent}>
                    <p>{msg.content}</p>

                    {/* Citations */}
                    {msg.citations && msg.citations.length > 0 && (
                      <div className={styles.citations}>
                        <p className={styles.citationsTitle}>📚 Sources:</p>
                        {msg.citations.map((citation, citIdx) => (
                          <a
                            key={citIdx}
                            href={citation.deep_link_url}
                            className={styles.citation}
                            target="_blank"
                            rel="noopener noreferrer"
                          >
                            {citation.section_title} ({citation.chunk_count})
                          </a>
                        ))}
                      </div>
                    )}

                    {/* Confidence Badge */}
                    {msg.confidence !== undefined && (
                      <div className={styles.metadata}>
                        <span className={styles.confidence}>
                          Confidence: {(msg.confidence * 100).toFixed(0)}%
                        </span>
                      </div>
                    )}
                  </div>
                  <div className={styles.timestamp}>
                    {msg.timestamp.toLocaleTimeString()}
                  </div>
                </div>
              ))
            )}

            {/* Loading Indicator */}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            {/* Error Message */}
            {error && (
              <div className={styles.errorBanner}>
                ⚠️ {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              maxLength={500}
              disabled={isLoading}
              className={styles.input}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className={styles.sendButton}
              aria-label="Send message"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>

          {/* Character Counter */}
          {inputValue.length > 0 && (
            <div className={styles.charCounter}>
              {inputValue.length}/500
            </div>
          )}
        </div>
      )}
    </>
  );
}
