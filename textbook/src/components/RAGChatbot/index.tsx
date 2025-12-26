import React, { useState, useEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ChatbotContent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState([]);
  const [loading, setLoading] = useState(false);
  const [portalTarget, setPortalTarget] = useState<HTMLElement | null>(null);
  const chatEndRef = useRef(null);

  const API_BASE_URL = 'https://physical-ai-backend-xnwe.onrender.com';

  // Build safety: document sirf browser mein milta hai
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

  const handleSearch = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setMessages(prev => [...prev, { role: 'user', text: query }]);
    setQuery('');
    setLoading(true);

    try {
      const res = await fetch(`${API_BASE_URL}/api/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query_text: query, selected_text: null }),
      });
      const data = await res.json();
      setMessages(prev => [...prev, { role: 'ai', text: data.answer }]);
    } catch (err) {
      setMessages(prev => [...prev, { role: 'ai', text: 'Error: Backend connection failed.' }]);
    } finally {
      setLoading(false);
    }
  };

  const ChatWindow = (
    <div style={{ 
      position: 'fixed', 
      bottom: '100px', 
      right: '25px', 
      width: '350px', 
      height: '500px', 
      backgroundColor: 'white', 
      borderRadius: '15px', 
      boxShadow: '0 10px 30px rgba(0,0,0,0.3)', 
      display: 'flex', 
      flexDirection: 'column', 
      zIndex: 999999,
      border: '1px solid #ddd'
    }}>
      <div style={{ backgroundColor: '#2e8555', color: 'white', padding: '15px', fontWeight: 'bold' }}>Physical AI Tutor</div>
      <div style={{ flex: 1, padding: '15px', overflowY: 'auto' }}>
        {messages.map((m, i) => (
          <div key={i} style={{ marginBottom: '10px', textAlign: m.role === 'user' ? 'right' : 'left' }}>
            <div style={{ display: 'inline-block', padding: '10px', borderRadius: '10px', backgroundColor: m.role === 'user' ? '#2e8555' : '#eee', color: m.role === 'user' ? 'white' : 'black' }}>
              {m.text}
            </div>
          </div>
        ))}
        {loading && <div style={{ fontSize: '12px', color: '#666' }}>Thinking...</div>}
        <div ref={chatEndRef} />
      </div>
      <form onSubmit={handleSearch} style={{ padding: '10px', borderTop: '1px solid #eee', display: 'flex' }}>
        <input value={query} onChange={(e) => setQuery(e.target.value)} style={{ flex: 1, padding: '8px' }} placeholder="Type..." />
        <button type="submit" style={{ marginLeft: '5px' }}>➔</button>
      </form>
    </div>
  );

  return (
    <div style={{ position: 'fixed', bottom: '25px', right: '25px', zIndex: 999999 }}>
      <button 
        onClick={() => setIsOpen(!isOpen)}
        style={{ width: '60px', height: '60px', borderRadius: '50%', backgroundColor: '#2e8555', color: 'white', border: 'none', cursor: 'pointer', fontSize: '24px' }}
      >
        {isOpen ? '✕' : '💬'}
      </button>
      {isOpen && portalTarget && ReactDOM.createPortal(ChatWindow, portalTarget)}
    </div>
  );
};

export default function RAGChatbot() {
  return <BrowserOnly>{() => <ChatbotContent />}</BrowserOnly>;
}