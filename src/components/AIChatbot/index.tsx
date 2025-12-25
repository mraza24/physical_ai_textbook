import React, { useState } from 'react';
import styles from './styles.module.css';

// 1. TypeScript Interfaces add karein
interface Citation {
  section_title: string;
  deep_link_url: string;
}

interface Message {
  text: string;
  sender: 'user' | 'ai';
  citations?: Citation[];
}

export default function AIChatbot(): JSX.Element {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [query, setQuery] = useState<string>('');
  // 2. State ko types assign karein
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState<boolean>(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim()) return;

    const userMessage: Message = { text: query, sender: 'user' };
    setMessages((prev) => [...prev, userMessage]);
    setLoading(true);
    setQuery('');

    try {
      // NOTE: Swagger mein aapka port 8001 tha, agar backend 8001 par hai toh yahan 8001 karein
      const response = await fetch('http://127.0.0.1:8001/api/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query_text: query }),
      });
      const data = await response.json();
      
      const aiMessage: Message = { 
        text: data.answer, 
        sender: 'ai', 
        citations: data.citations 
      };
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      setMessages((prev) => [...prev, { text: "Error connecting to AI backend.", sender: 'ai' }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? '✖' : '🤖 Ask Textbook AI'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>Physical AI Assistant</div>
          <div className={styles.chatBody}>
            {messages.map((msg, i) => (
              <div key={i} className={msg.sender === 'user' ? styles.userMsg : styles.aiMsg}>
                {msg.text}
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citationBox}>
                    <strong>Sources:</strong>
                    {msg.citations.map((c, idx) => (
                      <a key={idx} href={c.deep_link_url} className={styles.citationLink}>
                        {c.section_title}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {loading && <div className={styles.aiMsg}>Thinking...</div>}
          </div>
          <form onSubmit={handleSubmit} className={styles.chatInput}>
            <input 
              value={query} 
              onChange={(e) => setQuery(e.target.value)} 
              placeholder="Ask a question..." 
            />
            <button type="submit" disabled={loading}>Send</button>
          </form>
        </div>
      )}
    </div>
  );
}