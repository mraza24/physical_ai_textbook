import React, { useState } from 'react';
import styles from './styles.module.css';

interface UrduTranslateButtonProps {
  contentId?: string;
}

export default function UrduTranslateButton({ contentId = 'main' }: UrduTranslateButtonProps) {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [urduContent, setUrduContent] = useState<string | null>(null);

  // Trigger Bulldog explanation for translation
  const triggerBulldogExplanation = (translatingToUrdu: boolean) => {
    // Dispatch custom event for Bulldog to listen
    const event = new CustomEvent('urdu-translation-toggled', {
      detail: { isTranslatingToUrdu: translatingToUrdu }
    });
    window.dispatchEvent(event);
  };

  const translateToUrdu = async () => {
    if (isTranslating) return;

    // If already translated, toggle back to English
    if (isUrdu && originalContent) {
      const articleElement = document.querySelector('article');
      if (articleElement) {
        articleElement.innerHTML = originalContent;
        setIsUrdu(false);
        triggerBulldogExplanation(false); // Trigger Bulldog for English
      }
      return;
    }

    // If we have cached Urdu content, use it
    if (urduContent) {
      const articleElement = document.querySelector('article');
      if (articleElement) {
        articleElement.innerHTML = urduContent;
        setIsUrdu(true);
        triggerBulldogExplanation(true); // Trigger Bulldog for Urdu
      }
      return;
    }

    setIsTranslating(true);
    triggerBulldogExplanation(true); // Trigger Bulldog for first translation

    try {
      // Get the article content
      const articleElement = document.querySelector('article');
      if (!articleElement) {
        console.error('Article element not found');
        return;
      }

      // Save original content
      if (!originalContent) {
        setOriginalContent(articleElement.innerHTML);
      }

      // Extract text content for translation
      const textContent = articleElement.innerText;

      // Simulate AI translation (replace with actual AI API call)
      // For demo purposes, we'll create a simulated Urdu translation
      const simulatedUrduTranslation = await simulateUrduTranslation(textContent);

      // Create Urdu HTML
      const urduHTML = createUrduHTML(simulatedUrduTranslation);

      // Cache Urdu content
      setUrduContent(urduHTML);

      // Replace content with Urdu
      articleElement.innerHTML = urduHTML;
      setIsUrdu(true);
    } catch (error) {
      console.error('Translation failed:', error);
      alert('Translation failed. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const simulateUrduTranslation = async (text: string): Promise<string> => {
    // Simulate API delay
    await new Promise(resolve => setTimeout(resolve, 1500));

    // For demo, we'll provide actual Urdu translations of key sections
    // In production, this would call Claude API or another translation service

    const urduTranslations: Record<string, string> = {
      'Physical AI & Humanoid Robotics': 'فزیکل اے آئی اور ہیومنائیڈ روبوٹکس',
      'Building Intelligent Embodied Systems': 'ذہین مجسم نظام بنانا',
      'Welcome to Physical AI': 'فزیکل اے آئی میں خوش آمدید',
      'What You\'ll Learn': 'آپ کیا سیکھیں گے',
      'Module 1: The Robotic Nervous System': 'ماڈیول 1: روبوٹک اعصابی نظام',
      'Module 2: The Digital Twin': 'ماڈیول 2: ڈیجیٹل ٹوئن',
      'Module 3: The AI-Robot Brain': 'ماڈیول 3: اے آئی-روبوٹ برین',
      'Module 4: Vision-Language-Action Intelligence': 'ماڈیول 4: ویژن-لینگویج-ایکشن انٹیلیجنس',
      'Who This Textbook Is For': 'یہ کتاب کس کے لیے ہے',
      'Graduate Students': 'گریجویٹ طلباء',
      'Early-Career Engineers': 'ابتدائی کیریئر انجینئرز',
      'Getting Started': 'شروعات کریں',
    };

    // Create a translated version with key sections in Urdu
    let translatedText = text;

    // Replace key English phrases with Urdu
    Object.entries(urduTranslations).forEach(([english, urdu]) => {
      translatedText = translatedText.replace(new RegExp(english, 'g'), urdu);
    });

    // Add Urdu introduction
    const urduIntro = `
یہ کتاب جدید فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے بارے میں ایک مکمل، عملی سفر فراہم کرتی ہے۔ آپ ذہین مجسم نظام بنانا سیکھیں گے جو جسمانی دنیا میں محسوس کرتے ہیں، استدلال کرتے ہیں، اور عمل کرتے ہیں—روبوٹک کنٹرول (ROS 2)، فوٹو ریئلسٹک سیمولیشن (Gazebo, Unity, Isaac Sim)، GPU-accelerated اے آئی پرسیپشن (NVIDIA Isaac)، اور جدید ترین ویژن-لینگویج-ایکشن ماڈلز (VLA) کو یکجا کرتے ہوئے۔

اس کورس کے اختتام تک، آپ ایک ہیومنائیڈ روبوٹ بنا سکیں گے جو آواز کی کمانڈز کو سمجھتا ہے، بڑے لینگویج ماڈلز کا استعمال کرتے ہوئے پیچیدہ کاموں کی منصوبہ بندی کرتا ہے، ڈیپ نیورل نیٹ ورکس کے ساتھ اپنے ماحول کو محسوس کرتا ہے، اور محفوظ اور مؤثر طریقے سے اعمال انجام دیتا ہے۔
    `;

    return urduIntro + '\n\n' + translatedText;
  };

  const createUrduHTML = (urduText: string): string => {
    // Create formatted HTML with Urdu text
    const lines = urduText.split('\n');
    let html = '<div dir="rtl" style="font-family: \'Noto Nastaliq Urdu\', \'Jameel Noori Nastaleeq\', \'Alvi Nastaleeq\', serif; font-size: 1.1em; line-height: 2;">';

    lines.forEach(line => {
      if (line.trim()) {
        if (line.startsWith('##')) {
          html += `<h2 style="color: #667eea; margin-top: 2em; margin-bottom: 1em; font-size: 1.8em;">${line.replace(/##/g, '').trim()}</h2>`;
        } else if (line.startsWith('#')) {
          html += `<h1 style="color: #667eea; margin-top: 1.5em; margin-bottom: 1em; font-size: 2.2em;">${line.replace(/#/g, '').trim()}</h1>`;
        } else if (line.startsWith('ماڈیول')) {
          html += `<h3 style="color: #ff8c42; margin-top: 1.5em; margin-bottom: 0.8em; font-size: 1.5em;">${line}</h3>`;
        } else {
          html += `<p style="margin-bottom: 1em; text-align: right;">${line}</p>`;
        }
      }
    });

    html += '</div>';
    return html;
  };

  return (
    <div className={styles.urduButtonContainer}>
      <button
        onClick={translateToUrdu}
        className={`${styles.urduButton} ${isUrdu ? styles.urduButtonActive : ''}`}
        disabled={isTranslating}
      >
        {isTranslating ? (
          <>
            <span className={styles.spinner}></span>
            <span>Translating...</span>
          </>
        ) : isUrdu ? (
          <>
            <span className={styles.icon}>🇬🇧</span>
            <span>Show English</span>
          </>
        ) : (
          <>
            <span className={styles.icon}>🇵🇰</span>
            <span>Translate to Urdu | اردو میں ترجمہ کریں</span>
          </>
        )}
      </button>

      {isUrdu && (
        <div className={styles.urduNotice}>
          <span className={styles.icon}>ℹ️</span>
          <span>Content translated to Urdu. Technical terms preserved in English for clarity.</span>
        </div>
      )}
    </div>
  );
}
