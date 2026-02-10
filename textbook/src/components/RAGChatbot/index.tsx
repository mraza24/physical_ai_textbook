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

// Typewriter effect logic
const TypewriterText: React.FC<{ text: string; onComplete?: () => void }> = ({ text, onComplete }) => {
  const [displayedText, setDisplayedText] = useState('');
  
  useEffect(() => {
    let currentIndex = 0;
    const timer = setInterval(() => {
      if (currentIndex < text.length) {
        setDisplayedText(text.slice(0, currentIndex + 1));
        currentIndex++;
      } else {
        clearInterval(timer);
        if (onComplete) onComplete();
      }
    }, 15); // Faster typing
    return () => clearInterval(timer);
  }, [text]);

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

const ChatbotContent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const [portalTarget, setPortalTarget] = useState<HTMLElement | null>(null);
  const [typingComplete, setTypingComplete] = useState(true);
  const chatEndRef = useRef<HTMLDivElement>(null);

  // Unified Backend URL
  const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost' 
    ? 'http://localhost:4000' 
    : 'https://physical-ai-backend-xnwe.onrender.com';

  useEffect(() => {
    if (typeof document !== 'undefined') setPortalTarget(document.body);
  }, []);

  useEffect(() => {
    if (isOpen && chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, loading, isOpen]);

  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim() || !typingComplete) return;

    // CRITICAL: Block sending if not logged in - check BOTH flags
    if (typeof window !== 'undefined') {
      const isLoggedIn = localStorage.getItem('isLoggedIn');
      const authToken = localStorage.getItem('auth_token');

      if (!isLoggedIn || !authToken) {
        // Clean up stale flags
        localStorage.removeItem('isLoggedIn');
        localStorage.removeItem('just_logged_in');

        setMessages(prev => [...prev, {
          role: 'ai',
          text: 'Please login to chat with the Physical AI Tutor. 🤖'
        }]);
        setQuery('');
        return;
      }
    }

    const userMsg = query;
    setMessages(prev => [...prev, { role: 'user', text: userMsg }]);
    setQuery('');
    setLoading(true);
    setTypingComplete(false);

    try {
      const authToken = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
      const headers: HeadersInit = { 'Content-Type': 'application/json' };
      if (authToken) headers['Authorization'] = `Bearer ${authToken}`;

      // CRITICAL: Extract user level from localStorage profile
      let userLevel = 'Beginner'; // Default
      if (typeof window !== 'undefined') {
        const profileStr = localStorage.getItem('profile');
        if (profileStr) {
          try {
            const profile = JSON.parse(profileStr);
            if (profile.software_background) {
              userLevel = profile.software_background; // 'Beginner', 'Intermediate', or 'Expert'
              console.log('[RAGChatbot] Detected user level:', userLevel);
            }
          } catch (e) {
            console.error('[RAGChatbot] Failed to parse profile:', e);
          }
        }
      }

      const res = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          query_text: userMsg,
          user_level: userLevel // Send user level to backend
        }),
      });

      if (!res.ok) throw new Error(`HTTP ${res.status}`);

      const data = await res.json();
      setMessages(prev => [...prev, {
        role: 'ai',
        text: data.answer || data.text || "I'm processing that...",
        sources: data.citations || [],
        isTyping: true
      }]);
    } catch (err) {
      setMessages(prev => [...prev, {
        role: 'ai',
        text: "⚠️ Backend connection failed. Please ensure your server is running."
      }]);
      setTypingComplete(true);
    } finally {
      setLoading(false);
    }
  };

  const ChatWindow = (
    <div className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}>
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <div className={styles.headerIcon}>🤖</div>
          <div className={styles.headerText}>
            <div className={styles.headerTitle}>Physical AI Tutor</div>
            <div className={styles.headerSubtitle}>Powered by RAG</div>
          </div>
        </div>
        <button onClick={() => setIsOpen(false)} className={styles.closeButton}>✕</button>
      </div>

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <div className={styles.welcomeIcon}>
              {typeof window !== 'undefined' && (!localStorage.getItem('isLoggedIn') || !localStorage.getItem('auth_token')) ? '🔒' : '👋'}
            </div>
            <h3>
              {typeof window !== 'undefined' && (!localStorage.getItem('isLoggedIn') || !localStorage.getItem('auth_token'))
                ? 'Login Required'
                : 'Welcome!'}
            </h3>
            <p>
              {typeof window !== 'undefined' && (!localStorage.getItem('isLoggedIn') || !localStorage.getItem('auth_token'))
                ? 'Please login to chat with the Physical AI Tutor. 🤖'
                : 'Ask me anything about Physical AI, robotics, or the textbook content.'}
            </p>
          </div>
        )}

        {messages.map((m, idx) => (
          <div key={idx} className={`${styles.messageWrapper} ${m.role === 'user' ? styles.userMessage : styles.aiMessage}`}>
            <div className={styles.messageBubble}>
              {m.isTyping && idx === messages.length - 1 
                ? <TypewriterText text={m.text} onComplete={() => setTypingComplete(true)} /> 
                : m.text}
            </div>
            {m.role === 'ai' && m.sources && <Citations sources={m.sources} />}
          </div>
        ))}
        {loading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.loadingDots}><span></span><span></span><span></span></div>
          </div>
        )}
        <div ref={chatEndRef} />
      </div>

      <form onSubmit={handleSearch} className={styles.inputForm}>
        <input 
          type="text" 
          value={query} 
          onChange={(e) => setQuery(e.target.value)} 
          className={styles.input} 
          placeholder="Ask about robotics..." 
          disabled={loading || !typingComplete} 
        />
        <button type="submit" className={styles.sendButton} disabled={!query.trim() || loading || !typingComplete}>
          ➤
        </button>
      </form>
    </div>
  );

  return (
    <div className={styles.chatbotContainer}>
      <button onClick={() => setIsOpen(!isOpen)} className={`${styles.fabButton} ${isOpen ? styles.fabButtonActive : ''}`}>
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