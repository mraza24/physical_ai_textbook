import React, { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { Dog } from 'lucide-react';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  text: string;
}

const BulldogAssistantContent = () => {
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '/';

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [userProfile, setUserProfile] = useState<any>(null);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
    if (typeof document !== 'undefined') {
      // CRITICAL: Check BOTH isLoggedIn flag AND auth_token to prevent session leaks
      const isLoggedIn = localStorage.getItem('isLoggedIn');
      const authToken = localStorage.getItem('auth_token');

      // If either is missing, clear stale flags and show login prompt
      if (!isLoggedIn || !authToken) {
        // Clean up stale login flags
        localStorage.removeItem('isLoggedIn');
        localStorage.removeItem('just_logged_in');
        console.log('[BulldogAssistant] Session invalid - cleared stale flags');

        // User not logged in - show login prompt only
        setMessages([{
          role: 'assistant',
          text: 'Please login to chat with Bulldog. 🐕'
        }]);
        return; // Exit early - no other welcome messages
      }

      // CRITICAL FIX: Load user profile from 'profile' key (matches login.tsx and ChapterActions.tsx)
      const profileStr = localStorage.getItem('profile');
      let loadedProfile = null;
      if (profileStr) {
        try {
          loadedProfile = JSON.parse(profileStr);
          setUserProfile(loadedProfile);
          console.log('[BulldogAssistant] Loaded user profile:', loadedProfile);
        } catch (e) {
          console.error('[BulldogAssistant] Failed to parse user profile:', e);
        }
      }

      // Check if user just logged in (Smart Chatbot Response - UX Requirement)
      const justLoggedIn = localStorage.getItem('just_logged_in');
      if (justLoggedIn === 'true') {
        localStorage.removeItem('just_logged_in'); // Clear flag
        setTimeout(() => {
          setIsOpen(true);

          // Smart login greeting with personalized recommendations
          const userName = loadedProfile?.name || 'there';
          const expertise = loadedProfile?.software_background || 'Beginner';
          const background = loadedProfile?.hardware_experience || 'Software';

          // Determine recommended starting chapter based on expertise
          let recommendedChapter = '';
          if (expertise === 'Beginner') {
            recommendedChapter = 'Chapter 1.1 (ROS 2 Fundamentals)';
          } else if (expertise === 'Intermediate') {
            recommendedChapter = 'Chapter 1.2 (ROS 2 Navigation)';
          } else {
            recommendedChapter = 'Chapter 3.2 (GPU-Accelerated Perception)';
          }

          setMessages([{
            role: 'assistant',
            text: `Welcome ${userName}! As a ${expertise} with a ${background} background, I recommend starting with ${recommendedChapter}. Click "Personalize" in any chapter to tailor the content for you! 🎯\n\nWoof! 🐕 I'm your Bulldog Assistant - ready to guide you through Physical AI and Robotics.\n\nLet's get started! 🚀`
          }]);
        }, 1000);
        return; // Don't show other welcome messages
      }

      // CRITICAL FIX: "Welcome Back" message for returning authenticated users
      // This runs when user is logged in (isLoggedIn=true) but NOT just logged in
      const hasSeenWelcomeBack = sessionStorage.getItem('bulldog_welcome_back_shown');
      if (!hasSeenWelcomeBack && loadedProfile) {
        sessionStorage.setItem('bulldog_welcome_back_shown', 'true');

        const userName = loadedProfile?.name || 'there';
        const currentPath = window.location.pathname;

        // Only show on homepage or docs pages, not on login/signup
        if (currentPath.includes('/docs/') || currentPath === baseUrl || currentPath === `${baseUrl}`) {
          setMessages([{
            role: 'assistant',
            text: `Welcome back, ${userName}! 🐕\n\nYour personalization and translation features are active across all 16 chapters.\n\nFeel free to:\n• 🎯 Click "Personalize" to adapt content to your level\n• 🌍 Click "Translate to Urdu" for multilingual support\n• 💬 Ask me anything about the chapters\n\nReady to continue learning! 🚀`
          }]);
          console.log('[BulldogAssistant] Showed "Welcome Back" message for returning user');
        }
      }

      // Auto-welcome with personalized greeting (Task 5 & 6)
      const currentPath = window.location.pathname;
      const fromAIPowered = localStorage.getItem('from_ai_powered');
      const autoOpenBulldog = localStorage.getItem('auto_open_bulldog');

      if (currentPath.includes('/docs/intro') && messages.length === 0) {
        // Force open if auto_open_bulldog flag is set
        const shouldAutoOpen = autoOpenBulldog === 'true';

        // Only show welcome if it's a natural visit OR forced by AI-Powered button
        if (shouldAutoOpen || !fromAIPowered) {
          if (shouldAutoOpen) {
            localStorage.removeItem('auto_open_bulldog');
          }

          setTimeout(() => {
            setIsOpen(shouldAutoOpen); // Force open only if flag is set

            // Task 5 & 6: Personalized greeting with user profile (use loadedProfile for immediate access)
            const userName = loadedProfile?.name || 'there';
            const expertise = loadedProfile?.software_background || 'Beginner';
            const background = loadedProfile?.hardware_experience || 'Software';

            // Determine recommended chapters based on profile
            let recommendedChapters = '';
            if (expertise === 'Beginner') {
              recommendedChapters = 'Chapter 1.1 (ROS 2 Fundamentals) and Chapter 2.1 (Gazebo Simulation)';
            } else if (expertise === 'Intermediate') {
              recommendedChapters = 'Chapter 1.2 (ROS 2 Navigation) and Module 3 (NVIDIA Isaac)';
            } else {
              recommendedChapters = 'Chapter 3.2 (GPU-Accelerated Perception) and Module 4 (VLA Models)';
            }

            // Customize message based on source
            const welcomeText = fromAIPowered === 'true'
              ? `Welcome ${userName}! Since you are a ${expertise} in ${background}, I recommend you start with ${recommendedChapters}.\n\n🐕 I'm your Bulldog Assistant - your intelligent companion through this textbook!\n\nHere's what makes this textbook smart:\n• 🤖 Personalized content based on YOUR skill level\n• 🌍 Multilingual support (Urdu translation available!)\n• 💬 Ask me ANYTHING about robotics concepts\n• ✨ Interactive learning with real-time help\n\nClick the "🌍 Urdu Version" button above to see multilingual support in action!\n\nWhat would you like to learn about? 🚀`
              : `Welcome ${userName}! Since you are a ${expertise} in ${background}, I recommend you start with ${recommendedChapters}.\n\n🐕 I'm your Bulldog Assistant - ready to guide you through Physical AI!\n\nFeel free to ask me anything about:\n• ROS 2 basics\n• Learning path recommendations\n• Hardware requirements\n• Any chapter topics\n\nLet's learn together! 🤖`;

            setMessages([{
              role: 'assistant',
              text: welcomeText
            }]);

            // Clear the flag after showing message
            if (fromAIPowered === 'true') {
              localStorage.removeItem('from_ai_powered');
            }
          }, 1500); // Delay for smooth animation
        }
      }

      // Listen for personalization/translation events (Task 6 & 7)
      const handleBulldogNotify = (event: CustomEvent) => {
        const { message, type } = event.detail;

        // CRITICAL FIX: If this is a translation event, trigger the actual content swap
        if (type === 'translation') {
          console.log('[BulldogAssistant] Translation event detected - triggering content swap');
          // Dispatch event to ChapterActions to actually toggle the content
          window.dispatchEvent(new CustomEvent('translate:trigger', {
            detail: { action: 'enable' }
          }));
        }

        setTimeout(() => {
          setIsOpen(true);
          setMessages(prev => [...prev, {
            role: 'assistant',
            text: message
          }]);
        }, 500);
      };

      // Legacy Urdu translation event handler
      const handleUrduTranslation = (event: CustomEvent) => {
        const { isTranslatingToUrdu } = event.detail;

        setTimeout(() => {
          setIsOpen(true);

          const explanationMessage = isTranslatingToUrdu
            ? `Woof! 🐕 I've translated this chapter to Urdu for you because I'm your AI Agent!\n\nAs your intelligent assistant, I can help make this content accessible in multiple languages. Urdu translation allows students from Pakistan and other Urdu-speaking regions to learn robotics in their native language.\n\nThis is part of Task 7 - demonstrating how AI agents can break language barriers in education! 🌍\n\nNeed help understanding any robotics concepts? Just ask! 🤖`
            : `Woof! 🐕 I've switched the content back to English for you!\n\nAs your AI Agent, I'm here to make learning flexible and accessible. You can toggle between English and Urdu anytime.\n\nLet me know if you have questions about the content! 🤖`;

          setMessages(prev => [...prev, {
            role: 'assistant',
            text: explanationMessage
          }]);
        }, 500); // Small delay for smooth animation
      };

      window.addEventListener('bulldog:notify', handleBulldogNotify as EventListener);
      window.addEventListener('urdu-translation-toggled', handleUrduTranslation as EventListener);

      // Cleanup
      return () => {
        window.removeEventListener('bulldog:notify', handleBulldogNotify as EventListener);
        window.removeEventListener('urdu-translation-toggled', handleUrduTranslation as EventListener);
      };
    }
  }, []);

  const handleSend = async () => {
    if (!input.trim() || loading) return;

    // CRITICAL: Block sending if not logged in - check BOTH flags
    if (typeof window !== 'undefined') {
      const isLoggedIn = localStorage.getItem('isLoggedIn');
      const authToken = localStorage.getItem('auth_token');

      if (!isLoggedIn || !authToken) {
        // Clean up stale flags
        localStorage.removeItem('isLoggedIn');
        localStorage.removeItem('just_logged_in');

        setMessages(prev => [...prev, {
          role: 'assistant',
          text: 'Please login to chat with Bulldog. 🐕'
        }]);
        return;
      }
    }

    const userMessage = input;
    setInput('');
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setLoading(true);

    try {
      // Personalized response based on user's background
      const softwareLevel = userProfile?.software_background || 'Beginner';
      const hardwareLevel = userProfile?.hardware_experience || 'None';

      // Simulate personalized AI response
      const personalizedResponse = getPersonalizedResponse(userMessage, softwareLevel, hardwareLevel, baseUrl);

      await new Promise(resolve => setTimeout(resolve, 800)); // Simulate API delay

      setMessages(prev => [...prev, { role: 'assistant', text: personalizedResponse }]);
    } catch (error) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        text: "Woof! I'm having trouble connecting right now. Please try again!"
      }]);
    } finally {
      setLoading(false);
    }
  };

  // Helper function to convert markdown links to HTML
  const renderMessageWithLinks = (text: string) => {
    // Convert markdown links [text](url) to HTML <a> tags
    const parts = text.split(/(\[.*?\]\(.*?\))/g);

    return parts.map((part, idx) => {
      const linkMatch = part.match(/\[(.*?)\]\((.*?)\)/);
      if (linkMatch) {
        const [, linkText, url] = linkMatch;
        return (
          <a key={idx} href={url} style={{ color: '#667eea', textDecoration: 'underline', fontWeight: 'bold' }}>
            {linkText}
          </a>
        );
      }
      // Split by \n and render line breaks
      return part.split('\n').map((line, lineIdx) => (
        <React.Fragment key={`${idx}-${lineIdx}`}>
          {line}
          {lineIdx < part.split('\n').length - 1 && <br />}
        </React.Fragment>
      ));
    });
  };

  const getPersonalizedResponse = (question: string, softwareLevel: string, hardwareLevel: string, baseUrl: string): string => {
    const lowerQ = question.toLowerCase();

    // Personalized greetings
    if (lowerQ.includes('hello') || lowerQ.includes('hi') || lowerQ.includes('hey')) {
      return `Woof woof! 🐕 I'm your Bulldog Assistant! I see you're at a ${softwareLevel} software level and ${hardwareLevel} hardware experience. How can I help you with Physical AI today?`;
    }

    // ROS 2 questions
    if (lowerQ.includes('ros') || lowerQ.includes('robot operating system')) {
      if (softwareLevel === 'Beginner') {
        return `Woof! For beginners like you, ROS 2 is a framework that helps robots communicate. Think of it like a messaging app for robot parts! Start with Module 1: The Robotic Nervous System. I recommend starting with simple pub/sub examples. 🤖`;
      } else if (softwareLevel === 'Intermediate') {
        return `Great question! ROS 2 uses DDS for real-time communication. Check out Module 1 for advanced patterns like action servers and lifecycle nodes. You're ready for more complex topics! 🚀`;
      } else {
        return `Excellent! As an expert, you'll appreciate ROS 2's QoS policies and multi-threaded executors. Module 1 covers advanced topics like real-time control and custom interfaces. 💪`;
      }
    }

    // Hardware questions
    if (lowerQ.includes('hardware') || lowerQ.includes('sensor') || lowerQ.includes('actuator')) {
      if (hardwareLevel === 'None' || hardwareLevel === 'Basic') {
        return `Woof! Starting with hardware? Module 2 (Digital Twins) is perfect - you can learn in simulation first! Use Gazebo to practice before touching real hardware. Much safer! 🛠️`;
      } else {
        return `Nice! With your advanced hardware experience, check out Module 3 on NVIDIA Isaac for hardware-accelerated robotics. You'll love the real-time control sections! ⚡`;
      }
    }

    // VLA questions
    if (lowerQ.includes('vla') || lowerQ.includes('vision') || lowerQ.includes('language')) {
      return `Vision-Language-Action models are cutting-edge! Module 4 covers this. They let robots understand visual scenes AND natural language commands. ${softwareLevel === 'Expert' ? 'Perfect for your level!' : 'Start with the basics in earlier modules first!'} 🧠`;
    }

    // Learning path - with clickable links
    if (lowerQ.includes('start') || lowerQ.includes('begin') || lowerQ.includes('learn') || lowerQ.includes('how')) {
      const firstChapterLink = `${baseUrl}docs/module1/chapter1-1-ros2-fundamentals`;
      const module1Link = `${baseUrl}docs/module1/intro`;

      if (softwareLevel === 'Beginner') {
        return `Woof! Here's your personalized learning path:\n\n📚 Start here: Click to go to the first chapter → [ROS 2 Fundamentals](${firstChapterLink})\n\n1️⃣ Module 1: ROS 2 Basics → [Start Module 1](${module1Link})\n2️⃣ Module 2: Gazebo Simulation\n3️⃣ Then move to Modules 3 & 4\n\nTake your time, practice each chapter! 🎓`;
      } else {
        return `Based on your ${softwareLevel} level, I recommend:\n\n📚 Quick start: [ROS 2 Fundamentals](${firstChapterLink})\n\n1️⃣ Review Module 1 fundamentals → [Start here](${module1Link})\n2️⃣ Deep dive into Module 3 (NVIDIA Isaac)\n3️⃣ Master Module 4 (VLA models)\n\nYou've got this! 💪`;
      }
    }

    // Default personalized response
    return `Woof! I'm here to help with your Physical AI journey! Your background: ${softwareLevel} software, ${hardwareLevel} hardware. Try asking about:\n• ROS 2 concepts\n• Hardware setup\n• Learning path\n• Specific modules 🐕`;
  };

  const ChatWindow = (
    <div className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}>
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <Dog size={24} className={styles.headerIcon} />
          <div className={styles.headerText}>
            <div className={styles.headerTitle}>Bulldog Assistant</div>
            <div className={styles.headerSubtitle}>
              {userProfile ? `${userProfile.software_background} Level` : 'Your Personal Guide'}
            </div>
          </div>
        </div>
        <button onClick={() => setIsOpen(false)} className={styles.closeButton}>✕</button>
      </div>

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <Dog size={48} className={styles.welcomeIcon} />
            <h3>Woof! 🐕</h3>
            <p>
              I'm your personalized Bulldog Assistant!
              <br />
              Ask me about Physical AI, ROS 2, or your learning path.
            </p>
          </div>
        )}

        {messages.map((msg, idx) => (
          <div key={idx} className={`${styles.messageWrapper} ${msg.role === 'user' ? styles.userMessage : styles.assistantMessage}`}>
            <div className={styles.messageBubble}>
              {renderMessageWithLinks(msg.text)}
            </div>
          </div>
        ))}

        {loading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
      </div>

      <form onSubmit={(e) => { e.preventDefault(); handleSend(); }} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          className={styles.input}
          placeholder="Ask me anything..."
          disabled={loading}
        />
        <button type="submit" className={styles.sendButton} disabled={!input.trim() || loading}>
          ➤
        </button>
      </form>
    </div>
  );

  // Don't render until mounted (client-side only)
  if (!mounted) {
    return null;
  }

  // Portal content - Bulldog Assistant UI
  const bulldogContent = (
    <div className={styles.bulldogContainer}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`${styles.fabButton} ${isOpen ? styles.fabButtonActive : ''}`}
      >
        <Dog size={28} className={styles.fabIcon} />
        {!isOpen && <span className={styles.fabPulse}></span>}
      </button>
      {isOpen && ChatWindow}
    </div>
  );

  // Use React Portal to render directly to document.body
  // This completely bypasses the Docusaurus component tree
  return createPortal(bulldogContent, document.body);
};

export default function BulldogAssistant() {
  return (
    <div style={{ width: 0, height: 0, overflow: 'visible', pointerEvents: 'none' }}>
      <BrowserOnly fallback={null}>
        {() => <BulldogAssistantContent />}
      </BrowserOnly>
    </div>
  );
}
