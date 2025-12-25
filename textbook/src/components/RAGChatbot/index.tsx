import React, { useState, useEffect, useRef } from 'react';

const API_BASE_URL = 'http://localhost:8000';

interface Message {
  role: 'user' | 'ai';
  text: string;
  sources?: any[];
}

const RAGChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const chatEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, loading]);

  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim()) return;

    const userMessage: Message = { role: 'user', text: query };
    setMessages(prev => [...prev, userMessage]);
    setQuery('');
    setLoading(true);

    try {
      const res = await fetch(`${API_BASE_URL}/api/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query_text: query, selected_text: null }),
      });

      if (!res.ok) throw new Error();
      const data = await res.json();
      
      const aiMessage: Message = { 
        role: 'ai', 
        text: data.answer, 
        sources: data.citations 
      };
      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      setMessages(prev => [...prev, { 
        role: 'ai', 
        text: 'Connection Error: Could not reach the backend. Please ensure Port 8000 is running. | رابطہ منقطع: براہ کرم چیک کریں کہ سرور چل رہا ہے۔' 
      }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ position: 'fixed', bottom: '25px', right: '25px', zIndex: 10000, fontFamily: 'sans-serif' }}>
      
      <button 
        onClick={() => setIsOpen(!isOpen)}
        style={{
          width: '60px', height: '60px', borderRadius: '50%', backgroundColor: '#2e8555',
          color: 'white', border: 'none', cursor: 'pointer', boxShadow: '0 4px 15px rgba(0,0,0,0.3)',
          fontSize: '24px', transition: '0.3s'
        }}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {isOpen && (
        <div style={{
          position: 'absolute', bottom: '80px', right: '0', width: '380px', height: '520px',
          backgroundColor: 'white', borderRadius: '15px', boxShadow: '0 10px 30px rgba(0,0,0,0.2)',
          display: 'flex', flexDirection: 'column', overflow: 'hidden', border: '1px solid #eee'
        }}>
          
          <div style={{ backgroundColor: '#2e8555', color: 'white', padding: '15px 20px' }}>
            <h3 style={{ margin: 0, fontSize: '18px' }}>Textbook AI Assistant</h3>
            <span style={{ fontSize: '12px', opacity: 0.8 }}>Ask about Physical AI | فزیکل اے آئی کے بارے میں پوچھیں</span>
          </div>

          <div style={{ flex: 1, padding: '15px', overflowY: 'auto', backgroundColor: '#f8f9fa' }}>
            {messages.length === 0 && (
              <div style={{ textAlign: 'center', color: '#888', marginTop: '35%' }}>
                <p style={{ marginBottom: '10px' }}>👋 Hello! I am your AI tutor. Ask me any question.</p>
                <p dir="rtl">سلام! میں آپ کا اے آئی ٹیوٹر ہوں۔ کوئی بھی سوال پوچھیں۔</p>
              </div>
            )}
            
            {messages.map((m, i) => (
              <div key={i} style={{ 
                display: 'flex', 
                justifyContent: m.role === 'user' ? 'flex-end' : 'flex-start',
                marginBottom: '15px' 
              }}>
                <div style={{
                  maxWidth: '85%', padding: '10px 14px', borderRadius: '15px',
                  fontSize: '14px', lineHeight: '1.5',
                  backgroundColor: m.role === 'user' ? '#2e8555' : '#ffffff',
                  color: m.role === 'user' ? 'white' : '#333',
                  boxShadow: '0 2px 5px rgba(0,0,0,0.05)',
                  borderBottomRightRadius: m.role === 'user' ? '2px' : '15px',
                  borderBottomLeftRadius: m.role === 'ai' ? '2px' : '15px'
                }}>
                  {m.text}
                  {m.sources && m.sources.length > 0 && (
                    <div style={{ marginTop: '8px', borderTop: '1px solid #ddd', paddingTop: '5px', fontSize: '11px', color: '#666' }}>
                      <strong>Sources:</strong> {m.sources.map((s: any) => s.source || s.page).join(', ')}
                    </div>
                  )}
                </div>
              </div>
            ))}
            {loading && <div style={{ fontSize: '12px', color: '#2e8555', paddingLeft: '10px' }}>AI is thinking... | اے آئی سوچ رہا ہے</div>}
            <div ref={chatEndRef} />
          </div>

          <form onSubmit={handleSearch} style={{ padding: '15px', borderTop: '1px solid #eee', display: 'flex', gap: '8px' }}>
            <input 
              type="text" 
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              placeholder="Ask a question / سوال پوچھیں..." 
              style={{ 
                flex: 1, padding: '12px', borderRadius: '25px', 
                border: '1px solid #ddd', outline: 'none', fontSize: '14px' 
              }}
            />
            <button 
              type="submit" 
              disabled={loading}
              style={{ 
                backgroundColor: '#2e8555', color: 'white', border: 'none', 
                width: '40px', height: '40px', borderRadius: '50%', cursor: 'pointer',
                opacity: loading ? 0.6 : 1
              }}
            >
              ➔
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;