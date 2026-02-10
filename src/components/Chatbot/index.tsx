import React, { useState, useRef, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

const API_BASE_URL = 'http://localhost:4000/api/chat'; // Aapka local backend URL

function ChatbotComponent() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [typingText, setTypingText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, typingText]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    const query = inputValue.trim();
    if (!query || isLoading) return;

    setMessages(prev => [...prev, { role: 'user', content: query }]);
    setInputValue('');
    setIsLoading(true);

    try {
      // NOTE: Ensure your backend uses POST if that's what we set up earlier
      const response = await fetch(API_BASE_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query_text: query })
      });
      
      const data = await response.json();
      
      if (data.answer) {
        simulateTypewriter(data.answer);
      } else {
        throw new Error("No answer found");
      }
    } catch (err) {
      setMessages(prev => [...prev, { role: 'assistant', content: "⚠️ Connection error. Please check if your backend is running on port 4000." }]);
      setIsLoading(false);
    }
  };

  const simulateTypewriter = (fullText) => {
    let i = 0;
    setTypingText('');
    const interval = setInterval(() => {
      setTypingText(prev => prev + fullText.charAt(i));
      i++;
      if (i >= fullText.length) {
        clearInterval(interval);
        setMessages(prev => [...prev, { role: 'assistant', content: fullText }]);
        setTypingText('');
        setIsLoading(false);
      }
    }, 15); 
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Pulse FAB */}
      <button className={`${styles.fab} ${isOpen ? styles.fabActive : ''}`} onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? '✕' : '🤖'}
        {!isOpen && <div className={styles.pulseRing}></div>}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.header}>
            <div className={styles.headerIcon}>🤖</div>
            <div className={styles.headerText}>
              <div className={styles.headerTitle}>Physical AI Tutor</div>
              <div className={styles.headerSubtitle}>Always online</div>
            </div>
          </div>

          <div className={styles.messagesList}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <h3>Welcome! 👋</h3>
                <p>Ask me anything about ROS2, Robotics, or AI.</p>
              </div>
            )}
            
            {messages.map((msg, i) => (
              <div key={i} className={`${styles.messageWrapper} ${msg.role === 'user' ? styles.userMessage : styles.aiMessage}`}>
                <div className={styles.bubble}>
                  {msg.content}
                </div>
              </div>
            ))}

            {typingText && (
              <div className={`${styles.messageWrapper} styles.aiMessage`}>
                <div className={styles.bubble}>
                  {typingText}
                </div>
              </div>
            )}

            {isLoading && !typingText && (
              <div className={styles.loadingDots}>
                <span></span><span></span><span></span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputArea}>
            <input
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Type your message..."
              disabled={isLoading}
            />
            <button className={styles.sendButton} type="submit" disabled={isLoading || !inputValue.trim()}>
              ➤
            </button>
          </form>
        </div>
      )}
    </div>
  );
}

export default function Chatbot() {
  return <BrowserOnly>{() => <ChatbotComponent />}</BrowserOnly>;
}