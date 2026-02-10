import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

function ChatbotContent() {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState([]);
  const [loading, setLoading] = useState(false);
  const [currentTypingIndex, setCurrentTypingIndex] = useState(null);
  const chatEndRef = useRef(null);
  const inputRef = useRef(null);

  const API_BASE_URL = 'https://khans-rag-project-rag-api.hf.space/chat';

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (isOpen && chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, loading, isOpen]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  }, [isOpen]);

  // Typewriter effect for AI messages
  useEffect(() => {
    if (currentTypingIndex === null) return;

    const message = messages[currentTypingIndex];
    if (!message || message.role !== 'ai' || !message.isTyping) return;

    const fullText = message.fullText || message.text;
    const currentLength = message.text.length;

    if (currentLength < fullText.length) {
      const timer = setTimeout(() => {
        setMessages((prev) =>
          prev.map((msg, idx) =>
            idx === currentTypingIndex
              ? { ...msg, text: fullText.slice(0, currentLength + 1) }
              : msg
          )
        );
      }, 20);

      return () => clearTimeout(timer);
    } else {
      setMessages((prev) =>
        prev.map((msg, idx) =>
          idx === currentTypingIndex ? { ...msg, isTyping: false } : msg
        )
      );
      setCurrentTypingIndex(null);
    }
  }, [messages, currentTypingIndex]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!query.trim() || loading || currentTypingIndex !== null) return;

    const userMessage = { role: 'user', text: query };
    setMessages((prev) => [...prev, userMessage]);
    setQuery('');
    setLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}?query=${encodeURIComponent(query)}`, {
        method: 'GET',
        headers: { 'Accept': 'application/json' },
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }

      const data = await response.json();

      const aiMessage = {
        role: 'ai',
        text: '',
        fullText: data.answer || 'No response received.',
        sources: data.citations || [],
        isTyping: true,
      };

      setMessages((prev) => [...prev, aiMessage]);
      setCurrentTypingIndex(messages.length + 1);
    } catch (err) {
      console.error('Chat error:', err);
      const errorMessage = {
        role: 'ai',
        text: '⚠️ Connection failed. Please try again.',
        isTyping: false,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      {/* Floating Action Button */}
      <div className="chatbot-fab-container">
        <button
          className={`chatbot-fab ${isOpen ? 'chatbot-fab-active' : ''}`}
          onClick={() => setIsOpen(!isOpen)}
          aria-label={isOpen ? 'Close chat' : 'Open chat'}
        >
          <span className="chatbot-fab-icon">{isOpen ? '✕' : '💬'}</span>
          {!isOpen && <span className="chatbot-fab-pulse"></span>}
        </button>
      </div>

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-window">
          {/* Header */}
          <div className="chatbot-header">
            <div className="chatbot-header-content">
              <div className="chatbot-header-icon">🤖</div>
              <div className="chatbot-header-text">
                <div className="chatbot-header-title">Physical AI Tutor</div>
                <div className="chatbot-header-subtitle">Powered by RAG</div>
              </div>
            </div>
            <button
              className="chatbot-close-btn"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages Container */}
          <div className="chatbot-messages">
            {messages.length === 0 && (
              <div className="chatbot-welcome">
                <div className="chatbot-welcome-icon">👋</div>
                <h3>Welcome to Physical AI Tutor</h3>
                <p>Ask me anything about robotics, embodied AI, and autonomous systems!</p>
              </div>
            )}

            {messages.map((message, idx) => (
              <div
                key={idx}
                className={`chatbot-message-wrapper ${
                  message.role === 'user' ? 'chatbot-message-user' : 'chatbot-message-ai'
                }`}
              >
                <div className="chatbot-message-bubble">{message.text}</div>

                {message.role === 'ai' && message.sources && message.sources.length > 0 && (
                  <div className="chatbot-citations">
                    <div className="chatbot-citations-label">Sources:</div>
                    <div className="chatbot-citations-list">
                      {message.sources.map((source, sourceIdx) => (
                        <a
                          key={sourceIdx}
                          href={source.deep_link_url}
                          className="chatbot-citation-badge"
                          target="_blank"
                          rel="noopener noreferrer"
                        >
                          {source.section_title}
                        </a>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            ))}

            {loading && (
              <div className="chatbot-loading">
                <div className="chatbot-loading-dots">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={chatEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <input
              ref={inputRef}
              type="text"
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              className="chatbot-input"
              placeholder="Ask about humanoid robots, sensors, control systems..."
              disabled={loading || currentTypingIndex !== null}
            />
            <button
              type="submit"
              className="chatbot-send-btn"
              disabled={loading || currentTypingIndex !== null || !query.trim()}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                <path
                  d="M22 2L11 13"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
                <path
                  d="M22 2L15 22L11 13L2 9L22 2Z"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
            </button>
          </form>
        </div>
      )}
    </>
  );
}

export default function Chatbot() {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ChatbotContent />}
    </BrowserOnly>
  );
}
