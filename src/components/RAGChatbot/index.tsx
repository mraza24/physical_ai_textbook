/**
 * RAG Chatbot Component - SSG-Safe Version
 * Fully BrowserOnly-wrapped with inline styles
 */

import React, { useState, useRef, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

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

// Internal component that only renders client-side
function ChatbotInner() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Backend URL - safe to access window here since we're in BrowserOnly
  const API_BASE_URL = 'https://physical-ai-backend-xnwe.onrender.com/api/query';

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    const query = inputValue.trim();
    if (!query || isLoading) return;

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
      const response = await fetch(API_BASE_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query_text: query,
          selected_text: null,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();

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

      const errorMessage: ChatMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
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
      <style>{`
        .rag-chatbot-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 999999 !important;
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
        }

        .rag-floating-button {
          position: fixed;
          bottom: 20px;
          right: 20px;
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          border: none;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0,0,0,0.15);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: transform 0.2s, box-shadow 0.2s;
          z-index: 999999 !important;
          color: white;
        }

        .rag-floating-button:hover {
          transform: scale(1.05);
          box-shadow: 0 6px 20px rgba(0,0,0,0.2);
        }

        .rag-floating-button:active {
          transform: scale(0.95);
        }

        .rag-chat-window {
          position: fixed;
          bottom: 90px;
          right: 20px;
          width: 380px;
          height: 500px;
          background: white;
          border-radius: 12px;
          box-shadow: 0 8px 32px rgba(0,0,0,0.12);
          display: flex;
          flex-direction: column;
          z-index: 999999 !important;
          overflow: hidden;
          animation: slideUp 0.3s ease-out;
        }

        @keyframes slideUp {
          from {
            opacity: 0;
            transform: translateY(20px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }

        .rag-chat-header {
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          padding: 16px;
          display: flex;
          align-items: center;
          justify-content: space-between;
        }

        .rag-chat-header h3 {
          margin: 0;
          font-size: 16px;
          font-weight: 600;
        }

        .rag-header-actions {
          display: flex;
          gap: 8px;
        }

        .rag-clear-button,
        .rag-close-button {
          background: rgba(255,255,255,0.2);
          border: none;
          color: white;
          cursor: pointer;
          padding: 6px 10px;
          border-radius: 4px;
          font-size: 14px;
          transition: background 0.2s;
        }

        .rag-clear-button:hover,
        .rag-close-button:hover {
          background: rgba(255,255,255,0.3);
        }

        .rag-messages-container {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
          background: #f9fafb;
        }

        .rag-empty-state {
          text-align: center;
          color: #6b7280;
          padding: 20px;
        }

        .rag-empty-state p {
          margin: 8px 0;
        }

        .rag-example-questions {
          margin-top: 16px;
          display: flex;
          flex-direction: column;
          gap: 8px;
        }

        .rag-example-questions button {
          background: white;
          border: 1px solid #e5e7eb;
          padding: 10px;
          border-radius: 8px;
          cursor: pointer;
          text-align: left;
          font-size: 13px;
          transition: all 0.2s;
          color: #374151;
        }

        .rag-example-questions button:hover {
          border-color: #667eea;
          background: #f3f4f6;
        }

        .rag-message {
          margin-bottom: 12px;
          display: flex;
          flex-direction: column;
        }

        .rag-message.user .rag-message-content {
          background: #667eea;
          color: white;
          align-self: flex-end;
          border-radius: 12px 12px 0 12px;
        }

        .rag-message.assistant .rag-message-content {
          background: white;
          color: #1f2937;
          align-self: flex-start;
          border-radius: 12px 12px 12px 0;
          border: 1px solid #e5e7eb;
        }

        .rag-message-content {
          padding: 12px 16px;
          max-width: 85%;
          word-wrap: break-word;
        }

        .rag-message-content p {
          margin: 0;
          font-size: 14px;
          line-height: 1.5;
        }

        .rag-citations {
          margin-top: 12px;
          padding-top: 12px;
          border-top: 1px solid #e5e7eb;
        }

        .rag-citations-title {
          font-size: 12px;
          font-weight: 600;
          color: #6b7280;
          margin: 0 0 8px 0;
        }

        .rag-citation {
          display: block;
          font-size: 12px;
          color: #667eea;
          text-decoration: none;
          margin-bottom: 4px;
          transition: color 0.2s;
        }

        .rag-citation:hover {
          color: #764ba2;
          text-decoration: underline;
        }

        .rag-metadata {
          margin-top: 8px;
        }

        .rag-confidence {
          font-size: 11px;
          color: #6b7280;
          background: #f3f4f6;
          padding: 4px 8px;
          border-radius: 4px;
          display: inline-block;
        }

        .rag-timestamp {
          font-size: 11px;
          color: #9ca3af;
          margin-top: 4px;
        }

        .rag-loading-dots {
          display: flex;
          gap: 4px;
          padding: 12px 16px;
        }

        .rag-loading-dots span {
          width: 8px;
          height: 8px;
          background: #667eea;
          border-radius: 50%;
          animation: bounce 1.4s infinite ease-in-out both;
        }

        .rag-loading-dots span:nth-child(1) { animation-delay: -0.32s; }
        .rag-loading-dots span:nth-child(2) { animation-delay: -0.16s; }

        @keyframes bounce {
          0%, 80%, 100% { transform: scale(0); }
          40% { transform: scale(1); }
        }

        .rag-error-banner {
          background: #fee2e2;
          color: #991b1b;
          padding: 12px;
          border-radius: 8px;
          font-size: 13px;
          margin-bottom: 12px;
        }

        .rag-input-form {
          display: flex;
          padding: 16px;
          background: white;
          border-top: 1px solid #e5e7eb;
          gap: 8px;
        }

        .rag-input {
          flex: 1;
          padding: 10px 12px;
          border: 1px solid #d1d5db;
          border-radius: 8px;
          font-size: 14px;
          outline: none;
          transition: border-color 0.2s;
        }

        .rag-input:focus {
          border-color: #667eea;
        }

        .rag-input:disabled {
          background: #f3f4f6;
          cursor: not-allowed;
        }

        .rag-send-button {
          background: #667eea;
          color: white;
          border: none;
          padding: 10px 20px;
          border-radius: 8px;
          cursor: pointer;
          font-size: 16px;
          transition: background 0.2s;
        }

        .rag-send-button:hover:not(:disabled) {
          background: #5568d3;
        }

        .rag-send-button:disabled {
          background: #d1d5db;
          cursor: not-allowed;
        }

        .rag-char-counter {
          position: absolute;
          bottom: 72px;
          right: 36px;
          font-size: 11px;
          color: #9ca3af;
          background: white;
          padding: 2px 6px;
          border-radius: 4px;
        }

        @media (max-width: 480px) {
          .rag-chat-window {
            width: calc(100vw - 40px);
            height: calc(100vh - 120px);
            right: 20px;
            left: 20px;
            bottom: 90px;
          }
        }
      `}</style>

      <div className="rag-chatbot-container">
        {/* Floating Button */}
        <button
          className="rag-floating-button"
          onClick={() => setIsOpen(!isOpen)}
          aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
          type="button"
        >
          {isOpen ? (
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M18 6L6 18M6 6l12 12" strokeLinecap="round" />
            </svg>
          ) : (
            <span style={{ fontSize: '28px' }}>💬</span>
          )}
        </button>

        {/* Chat Window */}
        {isOpen && (
          <div className="rag-chat-window">
            {/* Header */}
            <div className="rag-chat-header">
              <h3>🤖 Ask the Textbook</h3>
              <div className="rag-header-actions">
                {messages.length > 0 && (
                  <button
                    onClick={handleClear}
                    className="rag-clear-button"
                    title="Clear chat"
                    type="button"
                  >
                    🗑️
                  </button>
                )}
                <button
                  onClick={() => setIsOpen(false)}
                  className="rag-close-button"
                  aria-label="Close chatbot"
                  type="button"
                >
                  ✕
                </button>
              </div>
            </div>

            {/* Messages */}
            <div className="rag-messages-container">
              {messages.length === 0 ? (
                <div className="rag-empty-state">
                  <p>👋 Hi! I'm your Physical AI textbook assistant.</p>
                  <p>Ask me anything about robotics, AI, or simulation!</p>
                  <div className="rag-example-questions">
                    <p><strong>Try asking:</strong></p>
                    <button type="button" onClick={() => setInputValue('What is forward kinematics?')}>
                      What is forward kinematics?
                    </button>
                    <button type="button" onClick={() => setInputValue('Explain inverse kinematics')}>
                      Explain inverse kinematics
                    </button>
                    <button type="button" onClick={() => setInputValue('What is ROS?')}>
                      What is ROS?
                    </button>
                  </div>
                </div>
              ) : (
                messages.map((msg, idx) => (
                  <div key={`${idx}-${msg.timestamp.getTime()}`} className={`rag-message ${msg.role}`}>
                    <div className="rag-message-content">
                      <p>{msg.content}</p>

                      {msg.citations && msg.citations.length > 0 && (
                        <div className="rag-citations">
                          <p className="rag-citations-title">📚 Sources:</p>
                          {msg.citations.map((citation, citIdx) => (
                            <a
                              key={citIdx}
                              href={citation.deep_link_url}
                              className="rag-citation"
                              target="_blank"
                              rel="noopener noreferrer"
                            >
                              {citation.section_title} ({citation.chunk_count})
                            </a>
                          ))}
                        </div>
                      )}

                      {msg.confidence !== undefined && (
                        <div className="rag-metadata">
                          <span className="rag-confidence">
                            Confidence: {(msg.confidence * 100).toFixed(0)}%
                          </span>
                        </div>
                      )}
                    </div>
                    <div className="rag-timestamp">
                      {msg.timestamp.toLocaleTimeString()}
                    </div>
                  </div>
                ))
              )}

              {isLoading && (
                <div className="rag-message assistant">
                  <div className="rag-loading-dots">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}

              {error && (
                <div className="rag-error-banner">
                  ⚠️ {error}
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {/* Input Form */}
            <form onSubmit={handleSubmit} className="rag-input-form">
              <input
                ref={inputRef}
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask a question..."
                maxLength={500}
                disabled={isLoading}
                className="rag-input"
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="rag-send-button"
                aria-label="Send message"
              >
                {isLoading ? '⏳' : '➤'}
              </button>
            </form>

            {inputValue.length > 0 && (
              <div className="rag-char-counter">
                {inputValue.length}/500
              </div>
            )}
          </div>
        )}
      </div>
    </>
  );
}

// Main export with BrowserOnly wrapper
export default function RAGChatbot() {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ChatbotInner />}
    </BrowserOnly>
  );
}
