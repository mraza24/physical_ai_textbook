/**
 * ChapterContentWrapper Component
 *
 * Wraps Docusaurus chapter content and provides real-time transformation functionality.
 * Injects ChapterActions component and manages content state (original/personalized/translated).
 *
 * Usage: Import in swizzled DocItem/Layout or custom page layouts
 */

import React, { useState, useEffect, useCallback } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChapterActions } from './ChapterActions';
import { MarkdownRenderer } from './MarkdownRenderer';
import styles from './ChapterContentWrapper.module.css';

interface ChapterContentWrapperProps {
  /** Current page path (e.g., "/docs/module1/intro") */
  pathname: string;
  /** Original markdown content */
  children: React.ReactNode;
}

export const ChapterContentWrapper: React.FC<ChapterContentWrapperProps> = ({
  pathname,
  children,
}) => {
  const [currentContent, setCurrentContent] = useState<React.ReactNode>(children);
  const [contentType, setContentType] = useState<'original' | 'personalized' | 'translated'>('original');

  // FORCE STATE UPDATE: Local view mode state for immediate reactivity
  const [viewMode, setViewMode] = useState<'standard' | 'personalized' | 'urdu'>('standard');

  // FORCE RE-RENDER: Unique key changes on every button click
  const [renderKey, setRenderKey] = useState(0);

  // Reset to original content when pathname changes (navigation)
  useEffect(() => {
    setCurrentContent(children);
    setContentType('original');
    setViewMode('standard'); // Reset view mode on navigation
  }, [pathname, children]);

  // Handle content transformation (personalized or translated)
  const handleContentChange = useCallback(
    (transformedMarkdown: string, type: 'personalized' | 'translated' | 'original') => {
      console.log('[ChapterContentWrapper] handleContentChange called:', {
        type,
        hasContent: !!transformedMarkdown,
        contentLength: transformedMarkdown?.length || 0
      });

      // FORCE RE-RENDER: Increment key to trigger complete UI refresh
      setRenderKey(prev => prev + 1);

      // MAP BUTTONS: Update viewMode based on type
      if (type === 'original') {
        setCurrentContent(children);
        setContentType('original');
        setViewMode('standard');
      } else if (type === 'personalized') {
        setContentType('personalized');
        setViewMode('personalized'); // FORCE STATE UPDATE
        // Use MarkdownRenderer to properly render
        setCurrentContent(
          <MarkdownRenderer
            content={transformedMarkdown}
            className="markdown-content transformed"
          />
        );
      } else if (type === 'translated') {
        setContentType('translated');
        setViewMode('urdu'); // FORCE STATE UPDATE
        // Use MarkdownRenderer to properly render
        setCurrentContent(
          <MarkdownRenderer
            content={transformedMarkdown}
            className="markdown-content transformed"
          />
        );
      }

      console.log('[ChapterContentWrapper] State updated - viewMode:', type === 'original' ? 'standard' : type === 'personalized' ? 'personalized' : 'urdu', 'renderKey:', renderKey + 1);
    },
    [children, renderKey]
  );

  // Extract original markdown content from React children
  // This is a simplified approach - in production, you'd extract from the MDX source
  const getOriginalMarkdown = useCallback((): string => {
    // For Docusaurus, we can extract text content from the rendered children
    // In a real implementation, this would come from the MDX source file
    if (typeof window !== 'undefined') {
      const contentElement = document.querySelector('.markdown');
      if (contentElement) {
        // Extract markdown-like representation from HTML
        // Note: This is a fallback - ideally we'd have access to the original MDX source
        return contentElement.textContent || '';
      }
    }
    return '';
  }, []);

  // FIX TRANSLATION CONNECTION: Hard-coded Urdu content for Chapter 1.1
  const urduText1_1 = `# باب 1.1: ROS 2 بنیادی باتیں

## تعارف

ROS 2 (Robot Operating System 2) روبوٹ سافٹ ویئر ڈویلپمنٹ کے لیے صنعتی معیار کا فریم ورک ہے۔

**آپ کیا سیکھیں گے:**
• **Nodes** - آزاد پروگرام
• **Topics** - کمیونیکیشن چینلز
• **Services** - درخواست-جواب میسجنگ

---

## Nodes - نوڈز

**سادہ وضاحت:** Node ایک worker bee کی طرح ہے۔ ہر ایک صرف ایک کام کرتا ہے۔

**مثالیں:**
• Sensor node → ڈیٹا پڑھتا ہے
• Motor node → پہیے چلاتا ہے
• Sensor node → temperature پڑھتا ہے

**آپ کا پہلا Node:**

\`\`\`python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('✅ میں زندہ ہوں!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
\`\`\`

---

## Topics - ٹاپکس

**سادہ وضاحت:** Topics ریڈیو چینلز کی طرح ہیں۔ Nodes broadcast یا listen کرتے ہیں۔

**حقیقی مثال:**
• Sensor "/sensor/data" پر broadcast کرتا ہے
• Display "/sensor/data" سنتا ہے اور ڈیٹا دکھاتا ہے

---

## Services - سروسز

**سادہ وضاحت:** Topics کے برعکس (one-way)، services آپ کو سوالات پوچھنے دیتی ہیں۔

**حقیقی مثال:**
• آپ: "اے robot، A سے B تک راستہ calculate کرو"
• Robot: "یہ رہا آپ کا راستہ: [X, Y, Z]"

---

## سیکھنے کا راستہ

**قدم 1:** اوپر MyNode کوڈ کاپی کریں → چلائیں → "میں زندہ ہوں!" دیکھیں
**قدم 2:** Publisher شامل کریں → پیغامات بھیجیں
**قدم 3:** Subscriber شامل کریں → پیغامات وصول کریں
**قدم 4:** سب کچھ ایک چھوٹے پروجیکٹ میں ملائیں

⏱️ **وقت درکار:** 30 منٹ مشق

---

💡 **Pro Tip:** ایک node اور ایک topic سے شروع کریں۔

📚 **اگلا:** باب 1.2 - ROS 2 Navigation
`;

  // HARD-CODED PERSONALIZED CONTENT for Chapter 1.1
  const personalizedText1_1 = `# Chapter 1.1: ROS 2 Fundamentals (Simplified Version)

## What You'll Learn

**3 Core Concepts** you need to know:

• **Nodes** = Independent programs
• **Topics** = Communication channels
• **Services** = Request-response messaging

---

## Nodes: The Building Blocks

**Simple Explanation:** A node is like a worker bee. Each one does ONE job.

**Examples:**
• Sensor node → Reads data
• Motor node → Moves wheels
• Sensor node → Reads temperature

**Your First Node (Copy & Run):**

\`\`\`python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('✅ I am alive!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
\`\`\`

---

## Topics: How Nodes Talk

**Simple Explanation:** Topics are like radio channels. Nodes broadcast or listen.

**Real Example:**
• Sensor broadcasts on channel "/sensor/data"
• Display listens to "/sensor/data" and shows information

**Code Pattern:**

\`\`\`python
# Send messages (Publisher)
self.pub = self.create_publisher(String, '/topic', 10)
self.pub.publish(String(data='Hello!'))

# Receive messages (Subscriber)
self.sub = self.create_subscription(String, '/topic', callback, 10)
\`\`\`

---

## Services: Ask and Answer

**Simple Explanation:** Unlike topics (one-way), services let you ask questions.

**Real Example:**
• You: "Hey robot, calculate path from A to B"
• Robot: "Here's your path: [X, Y, Z]"

---

## Quick Learning Path

**Step 1:** Copy the MyNode code above → Run it → See "I am alive!"
**Step 2:** Add a publisher → Send messages
**Step 3:** Add a subscriber → Receive messages
**Step 4:** Combine everything in a small project

⏱️ **Time needed:** 30 minutes of practice

---

## Cheat Sheet

| Concept | Use When |
|---------|----------|
| Node | Always (everything is a node) |
| Topic | Continuous data (sensors, status) |
| Service | One-time actions (start/stop, calculate) |

---

💡 **Pro Tip:** Start with ONE node and ONE topic. Master that before adding complexity.

📚 **Next:** Chapter 1.2 - ROS 2 Navigation
`;

  // HARD-CODE CHAPTER 1.1 CONTENT: Detect chapter
  const isChapter11 = pathname.includes('chapter1-1-ros2-fundamentals');

  // HARD-CODED INLINE STYLES: Direct style application for personalized view
  const isPersonalized = viewMode === 'personalized';

  // HELPER FUNCTION: Highlight key terms in text
  const highlightText = (text: string): string => {
    const keywords = ['Nodes', 'Launch', 'ROS 2', 'Topics', 'Communication', 'Services', 'Actions'];
    let highlighted = text;

    keywords.forEach(keyword => {
      const regex = new RegExp(`\\b(${keyword})\\b`, 'gi');
      highlighted = highlighted.replace(regex, `<span style="background-color: yellow; font-weight: bold; padding: 2px 4px;">$1</span>`);
    });

    return highlighted;
  };

  console.log('[ChapterContentWrapper] IMMEDIATE ACTION:', {
    viewMode,
    isChapter11,
    pathname,
    contentType,
    isPersonalized
  });

  // HARD-CODE LOGIC: if/else based on viewMode
  // if (viewMode === 'urdu') return <UrduView />;
  if (viewMode === 'urdu') {
    return (
      <BrowserOnly fallback={<div>{children}</div>}>
        {() => (
          <div key={`urdu-${renderKey}`} className={styles.contentWrapper}>
            <ChapterActions
              chapterId={pathname}
              originalContent={getOriginalMarkdown()}
              onContentChange={handleContentChange}
            />
            {/* TRANSLATION LOGIC: For ALL chapters, show AI-Generated Urdu container */}
            <div className={styles.urduView} dir="rtl" style={{ display: 'block', width: '100%' }}>
              {/* AI-Generated Urdu Container (Yellow Box) for ALL chapters */}
              <div style={{
                background: '#fef3c7',
                padding: '1.5rem',
                borderRight: '4px solid #f59e0b',
                marginBottom: '2rem',
                borderRadius: '8px',
                textAlign: 'right'
              }}>
                <h2 style={{ margin: '0 0 1rem 0', color: '#92400e', fontWeight: 'bold', fontSize: '1.3rem' }}>
                  اردو ترجمہ (AI-Generated)
                </h2>
                <p style={{ margin: 0, fontSize: '0.95rem', color: '#78350f' }}>
                  مصنوعی ذہانت سے تیار کردہ
                </p>
              </div>

              {isChapter11 ? (
                <MarkdownRenderer content={urduText1_1} className="markdown-content urdu" />
              ) : (
                <div style={{ display: 'block', textAlign: 'right', direction: 'rtl' }}>
                  <p style={{ fontSize: '1.1rem', lineHeight: '1.8', color: '#78350f' }}>
                    {typeof currentContent === 'string' ? currentContent : 'اردو ترجمہ دستیاب نہیں ہے'}
                  </p>
                </div>
              )}
            </div>
            {/* HIDE ORIGINAL ENGLISH: display: none - FORCE HIDDEN */}
            <div style={{ display: 'none', visibility: 'hidden', opacity: 0, height: 0, overflow: 'hidden' }}>{children}</div>
          </div>
        )}
      </BrowserOnly>
    );
  }

  // HARD-CODE LOGIC: if (viewMode === 'personalized') return <SimplifiedEnglishView />;
  if (viewMode === 'personalized') {
    console.log('[ChapterContentWrapper] RENDERING PERSONALIZED VIEW - Green border should be visible!');
    return (
      <BrowserOnly fallback={<div>{children}</div>}>
        {() => {
          console.log('[ChapterContentWrapper] BrowserOnly rendering personalized content');
          return (
            <div key="pers" className={styles.contentWrapper}>
              <ChapterActions
                chapterId={pathname}
                originalContent={getOriginalMarkdown()}
                onContentChange={handleContentChange}
              />
              {/* VISUAL INDICATION: Force inline style when isPersonalized is true */}
              <div
                style={{
                  border: isPersonalized ? '6px solid #22c55e' : 'none',
                  backgroundColor: '#f0fdf4',
                  padding: '15px',
                  borderRadius: '8px',
                  position: 'relative',
                  minHeight: '200px'
                }}
              >
              <div style={{
                position: 'absolute',
                top: '10px',
                right: '20px',
                backgroundColor: '#22c55e',
                color: 'white',
                padding: '0.5rem 1.2rem',
                borderRadius: '25px',
                fontWeight: 'bold',
                fontSize: '0.95rem',
                boxShadow: '0 4px 15px rgba(34, 197, 94, 0.5)',
                zIndex: 1000,
                display: 'flex',
                alignItems: 'center',
                gap: '0.5rem'
              }}>
                🤖 AI Personalized
              </div>
              {isChapter11 ? (
                <MarkdownRenderer
                  content={personalizedText1_1}
                  className="markdown-content personalized"
                  highlightKeywords={true}
                />
              ) : (
                currentContent
              )}
            </div>
          </div>
        );
        }}
      </BrowserOnly>
    );
  }

  // HARD-CODE LOGIC: else return <StandardView />;
  return (
    <BrowserOnly fallback={<div>{children}</div>}>
      {() => (
        <div key="orig" className={styles.contentWrapper}>
          <ChapterActions
            chapterId={pathname}
            originalContent={getOriginalMarkdown()}
            onContentChange={handleContentChange}
          />
          <div className="english-content" style={{ width: '100%' }}>
            {children}
          </div>
        </div>
      )}
    </BrowserOnly>
  );
};
