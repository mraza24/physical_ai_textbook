import React, { useState, useEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'ai';
  text: string;
  sources?: Citation[];
  isTyping?: boolean;
}

interface Citation {
  section_title: string;
  deep_link_url: string;
}

const useTypewriter = (text: string, speed: number = 30) => {
  const [displayedText, setDisplayedText] = useState('');
  const [isComplete, setIsComplete] = useState(false);

  useEffect(() => {
    setDisplayedText('');
    setIsComplete(false);
    let currentIndex = 0;

    const timer = setInterval(() => {
      if (currentIndex < text.length) {
        setDisplayedText(text.slice(0, currentIndex + 1));
        currentIndex++;
      } else {
        setIsComplete(true);
        clearInterval(timer);
      }
    }, speed);

    return () => clearInterval(timer);
  }, [text, speed]);

  return { displayedText, isComplete };
};

const TypewriterText: React.FC<{ text: string; onComplete?: () => void }> = ({ text, onComplete }) => {
  const { displayedText, isComplete } = useTypewriter(text, 20);

  useEffect(() => {
    if (isComplete && onComplete) {
      onComplete();
    }
  }, [isComplete, onComplete]);

  return <span>{displayedText}</span>;
};

const Citations: React.FC<{ sources: Citation[] }> = ({ sources }) => {
  if (!sources || sources.length === 0) return null;

  return (
    <div className={styles.citationsContainer}>
      <div className={styles.citationsLabel}>Sources:</div>
      <div className={styles.citationsList}>
        {sources.map((source, idx) => (
          <a key={idx} href={source.deep_link_url} className={styles.citationBadge} target="_blank" rel="noopener noreferrer">
            {source.section_title}
          </a>
        ))}
      </div>
    </div>
  );
};

const MessageBubble: React.FC<{ message: Message; isLatest: boolean; onTypingComplete: () => void }> = ({ message, isLatest, onTypingComplete }) => {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.messageWrapper} ${isUser ? styles.userMessage : styles.aiMessage}`}>
      <div className={styles.messageBubble}>
        {message.isTyping && isLatest ? <TypewriterText text={message.text} onComplete={onTypingComplete} /> : message.text}
      </div>
      {!isUser && message.sources && <Citations sources={message.sources} />}
    </div>
  );
};

const ChatbotContent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const [portalTarget, setPortalTarget] = useState<HTMLElement | null>(null);
  const [typingComplete, setTypingComplete] = useState(true);
  const chatEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const API_BASE_URL = 'http://localhost:4000';

  useEffect(() => {
    if (typeof document !== 'undefined') {
      setPortalTarget(document.body);
    }
  }, []);

  useEffect(() => {
    if (isOpen && chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, loading, isOpen]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim() || !typingComplete) return;

    // ✅ AUTH FIX: Get token from localStorage
    const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;

    const userMessage: Message = { role: 'user', text: query };
    setMessages(prev => [...prev, userMessage]);
    setQuery('');
    setLoading(true);
    setTypingComplete(false);

    try {
      const res = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers: { 
          'Content-Type': 'application/json',
          // ✅ AUTH FIX: Pass Bearer Token
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          query_text: query,
          selected_text: null
        }),
      });

      if (res.status === 401) {
        throw new Error('Unauthorized: Please login first.');
      }

      if (!res.ok) throw new Error(`Server Error: ${res.status}`);

      const data = await res.json();
      const aiMessage: Message = {
        role: 'ai',
        text: data.answer || 'No response received.',
        sources: data.citations || [],
        isTyping: true,
      };
      setMessages(prev => [...prev, aiMessage]);
    } catch (err: any) {
      setMessages(prev => [...prev, {
        role: 'ai',
        text: `⚠️ ${err.message || 'Connection failed'}. Check if backend is running on port 4000 and you are logged in.`,
        isTyping: false
      }]);
      setTypingComplete(true);
    } finally {
      setLoading(false);
    }
  };

  const handleTypingComplete = () => {
    setTypingComplete(true);
    setMessages(prev => {
      const updated = [...prev];
      if (updated.length > 0) {
        updated[updated.length - 1] = { ...updated[updated.length - 1], isTyping: false };
      }
      return updated;
    });
  };

  const ChatWindow = (
    <div className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}>
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <div className={styles.headerIcon}>🤖</div>
          <div className={styles.headerText}>
            <div className={styles.headerTitle}>Physical AI Tutor</div>
            <div className={styles.headerSubtitle}>Powered by Cohere + RAG</div>
          </div>
        </div>
        <button onClick={() => setIsOpen(false)} className={styles.closeButton} aria-label="Close chat">✕</button>
      </div>

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <div className={styles.welcomeIcon}>👋</div>
            <h3>Welcome to Physical AI Tutor</h3>
            <p>Ask me anything about robotics, sensors, and autonomous systems!</p>
          </div>
        )}

        {messages.map((message, idx) => (
          <MessageBubble key={idx} message={message} isLatest={idx === messages.length - 1} onTypingComplete={handleTypingComplete} />
        ))}

        {loading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.loadingDots}>
              <span></span><span></span><span></span>
            </div>
          </div>
        )}

        <div ref={chatEndRef} />
      </div>

      <form onSubmit={handleSearch} className={styles.inputForm}>
        <input
          ref={inputRef}
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className={styles.input}
          placeholder="Ask a question..."
          disabled={loading || !typingComplete}
        />
        <button type="submit" className={styles.sendButton} disabled={loading || !typingComplete || !query.trim()} aria-label="Send message">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      </form>
    </div>
  );

  return (
    <div className={styles.chatbotContainer}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`${styles.fabButton} ${isOpen ? styles.fabButtonActive : ''}`}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        <span className={styles.fabIcon}>{isOpen ? '✕' : '💬'}</span>
        {!isOpen && <span className={styles.fabPulse}></span>}
      </button>
      {isOpen && portalTarget && ReactDOM.createPortal(ChatWindow, portalTarget)}
    </div>
  );
};

export default function RAGChatbot() {
  return <BrowserOnly fallback={<div>Loading...</div>}>{() => <ChatbotContent />}</BrowserOnly>;
}