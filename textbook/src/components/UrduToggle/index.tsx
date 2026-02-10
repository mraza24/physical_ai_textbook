import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

// Urdu translation of intro page
const URDU_CONTENT = `
# جسمانی AI اور ہیومنائیڈ روبوٹکس

## ROS 2، ڈیجیٹل ٹوئنز، اور ویژن-لینگویج-ایکشن ماڈلز کے ساتھ ذہین مجسم نظام بنانا

---

**گریجویٹ طلباء اور روبوٹکس انجینئرز کے لیے ایک جامع درسی کتاب**

*روبوٹک کنٹرول، سمیولیشن، AI پرسیپشن، اور ملٹی موڈل انٹیلیجنس کو یکجا کرنا*

---

**ایڈیشن**: 1.0
**آخری اپڈیٹ**: دسمبر 2025
**ہدف سامعین**: گریجویٹ طلباء، ابتدائی کیریئر روبوٹکس انجینئرز
**تقاضے**: پروگرامنگ (Python/C++)، Linux بنیادی باتیں، کیلکولس، لکیری الجبرا

---

## جسمانی AI میں خوش آمدید

یہ درسی کتاب جدید جسمانی AI اور ہیومنائیڈ روبوٹکس کے ذریعے ایک مکمل، عملی سفر فراہم کرتی ہے۔ آپ ذہین مجسم نظام بنانا سیکھیں گے جو جسمانی دنیا میں سمجھتے، استدلال کرتے اور عمل کرتے ہیں - روبوٹک کنٹرول (ROS 2)، فوٹو ریئلسٹک سمیولیشن (Gazebo، Unity، Isaac Sim)، GPU سے تیز شدہ AI پرسیپشن (NVIDIA Isaac)، اور جدید ترین vision-language-action ماڈلز (VLA) کو ملا کر۔

اس کورس کے اختتام تک، آپ ایک ہیومنائیڈ روبوٹ بنانے کے قابل ہوں گے جو آواز کی کمانڈز کو سمجھتا ہے، بڑے لینگویج ماڈلز کا استعمال کرتے ہوئے پیچیدہ کاموں کی منصوبہ بندی کرتا ہے، اپنے ماحول کو گہرے نیورل نیٹ ورکس کے ساتھ محسوس کرتا ہے، اور محفوظ اور مؤثر طریقے سے کارروائیاں انجام دیتا ہے۔

---

## آپ کیا سیکھیں گے

### 🤖 **ماڈیول 1: روبوٹک اعصابی نظام**
ROS 2 میں مہارت حاصل کریں، روبوٹ سافٹویئر کے لیے صنعتی معیار کا فریم ورک۔ سیکھیں کہ nodes topics، services، اور actions کے ذریعے کیسے بات چیت کرتے ہیں۔ پیچیدہ ملٹی روبوٹ سسٹمز بنائیں، تعینات کریں اور منظم کریں۔

### 🌐 **ماڈیول 2: ڈیجیٹل ٹوئن**
Gazebo اور Unity کا استعمال کرتے ہوئے فوٹو ریئلسٹک ڈیجیٹل ٹوئنز بنائیں۔ سینسرز، فزکس، اور ماحول کو سمیولیٹ کریں۔ سمیولیشن سے حقیقی روبوٹس تک پالیسیاں تعینات کرنے کے لیے sim-to-real ٹرانسفر تکنیکوں کو سمجھیں۔

### 🧠 **ماڈیول 3: AI-روبوٹ برین**
GPU سے تیز شدہ پرسیپشن، نیویگیشن، اور manipulation کے لیے NVIDIA Isaac کا فائدہ اٹھائیں۔ TensorRT کے ساتھ گہرے نیورل نیٹ ورکس کو بہتر بنائیں۔ edge devices (Jetson Orin Nano) پر real-time inference کے لیے AI ماڈلز تعینات کریں۔

### 🎯 **ماڈیول 4: Vision-Language-Action انٹیلیجنس**
Vision-language-action ماڈلز (RT-1، RT-2، OpenVLA) کو بڑے لینگویج ماڈلز (GPT-4، Claude) اور speech recognition (Whisper) کے ساتھ یکجا کریں۔ end-to-end سسٹمز بنائیں جہاں روبوٹس قدرتی زبان کو سمجھتے ہیں، کاموں کے بارے میں استدلال کرتے ہیں، اور پیچیدہ manipulation انجام دیتے ہیں۔

---

## یہ درسی کتاب کس کے لیے ہے

### گریجویٹ طلباء
- Robotics، Computer Science، Electrical Engineering میں MS/PhD کر رہے ہیں
- روبوٹ کنٹرول، AI، کمپیوٹر ویژن، یا embodied intelligence میں کورسز لے رہے ہیں
- ہیومنائیڈ روبوٹس یا جسمانی AI سے متعلق تحقیقی منصوبوں پر کام کر رہے ہیں

### ابتدائی کیریئر انجینئرز
- روبوٹکس، خودمختار نظام، یا AI میں کیریئر شروع کر رہے ہیں
- حقیقی دنیا کی روبوٹ ایپلیکیشنز بنا رہے ہیں
- روایتی سافٹ ویئر سے embodied AI میں منتقل ہو رہے ہیں

### محققین اور پریکٹیشنرز
- VLA ماڈلز اور روبوٹکس کے لیے multimodal AI میں تازہ ترین پیش رفت تلاش کر رہے ہیں
- GPU سے تیز شدہ پرسیپشن پائپ لائنز نافذ کر رہے ہیں
- چیلنجنگ حقیقی دنیا کے ماحول میں روبوٹس تعینات کر رہے ہیں
`;

const ENGLISH_CONTENT = `
# Physical AI & Humanoid Robotics

## Building Intelligent Embodied Systems with ROS 2, Digital Twins, and Vision-Language-Action Models

---

**A Comprehensive Textbook for Graduate Students and Robotics Engineers**

*Integrating Robotic Control, Simulation, AI Perception, and Multimodal Intelligence*

---

**Edition**: 1.0
**Last Updated**: December 2025
**Target Audience**: Graduate students, early-career robotics engineers
**Prerequisites**: Programming (Python/C++), Linux basics, calculus, linear algebra

---

## Welcome to Physical AI

This textbook provides a complete, hands-on journey through modern physical AI and humanoid robotics. You'll learn to build intelligent embodied systems that perceive, reason, and act in the physical world—combining robotic control (ROS 2), photorealistic simulation (Gazebo, Unity, Isaac Sim), GPU-accelerated AI perception (NVIDIA Isaac), and cutting-edge vision-language-action models (VLA).

By the end of this course, you'll be able to create a humanoid robot that understands voice commands, plans complex tasks using large language models, perceives its environment with deep neural networks, and executes actions safely and efficiently.

---

## What You'll Learn

### 🤖 **Module 1: The Robotic Nervous System**
Master ROS 2, the industry-standard framework for robot software. Learn how nodes communicate via topics, services, and actions. Build, deploy, and manage complex multi-robot systems.

### 🌐 **Module 2: The Digital Twin**
Create photorealistic digital twins using Gazebo and Unity. Simulate sensors, physics, and environments. Understand sim-to-real transfer techniques to deploy policies from simulation to physical robots.

### 🧠 **Module 3: The AI-Robot Brain**
Leverage NVIDIA Isaac for GPU-accelerated perception, navigation, and manipulation. Optimize deep neural networks with TensorRT. Deploy AI models on edge devices (Jetson Orin Nano) for real-time inference.

### 🎯 **Module 4: Vision-Language-Action Intelligence**
Integrate vision-language-action models (RT-1, RT-2, OpenVLA) with large language models (GPT-4, Claude) and speech recognition (Whisper). Build end-to-end systems where robots understand natural language, reason about tasks, and execute complex manipulation.

---

## Who This Textbook Is For

### Graduate Students
- Pursuing MS/PhD in Robotics, Computer Science, Electrical Engineering
- Taking courses in robot control, AI, computer vision, or embodied intelligence
- Working on research projects involving humanoid robots or physical AI

### Early-Career Engineers
- Starting careers in robotics, autonomous systems, or AI
- Building real-world robot applications
- Transitioning from traditional software to embodied AI

### Researchers & Practitioners
- Exploring latest advances in VLA models and multimodal AI for robotics
- Implementing GPU-accelerated perception pipelines
- Deploying robots in challenging real-world environments
`;

function UrduToggleContent() {
  const [language, setLanguage] = useState<'english' | 'urdu'>('english');
  const [autoTriggered, setAutoTriggered] = useState(false);

  // Auto-trigger Urdu if ?translate=true or ?lang=ur in URL or localStorage flags
  React.useEffect(() => {
    if (typeof window === 'undefined' || autoTriggered) return;

    const params = new URLSearchParams(window.location.search);
    const shouldTranslate = params.get('translate') === 'true' ||
                           params.get('lang') === 'ur' ||
                           localStorage.getItem('trigger_urdu') === 'true' ||
                           localStorage.getItem('trigger_urdu_toc') === 'true';

    if (shouldTranslate) {
      setAutoTriggered(true);
      setLanguage('urdu');

      // Clear the localStorage flags
      localStorage.removeItem('trigger_urdu');
      localStorage.removeItem('trigger_urdu_toc');

      // Trigger Bulldog notification
      setTimeout(() => {
        const event = new CustomEvent('urdu-translation-toggled', {
          detail: { isTranslatingToUrdu: true }
        });
        window.dispatchEvent(event);
      }, 500);
    }
  }, [autoTriggered]);

  const toggleLanguage = () => {
    const newLanguage = language === 'english' ? 'urdu' : 'english';
    setLanguage(newLanguage);

    // Trigger Bulldog notification
    if (typeof window !== 'undefined') {
      const event = new CustomEvent('urdu-translation-toggled', {
        detail: { isTranslatingToUrdu: newLanguage === 'urdu' }
      });
      window.dispatchEvent(event);
    }
  };

  return (
    <div className={styles.urduToggleContainer}>
      {/* Big Visible Button at Top */}
      <div className={styles.buttonWrapper}>
        <button
          onClick={toggleLanguage}
          className={styles.urduButton}
          aria-label={language === 'english' ? 'Switch to Urdu' : 'Switch to English'}
        >
          <span className={styles.buttonIcon}>
            {language === 'english' ? '🌍' : '🔙'}
          </span>
          <span className={styles.buttonText}>
            {language === 'english' ? 'اردو ورژن (Urdu Version)' : 'English Version'}
          </span>
        </button>

        {language === 'urdu' && (
          <div className={styles.statusBadge}>
            ✓ Content translated to Urdu
          </div>
        )}
      </div>

      {/* Content Area */}
      <div className={styles.contentArea}>
        {language === 'urdu' ? (
          <div className={styles.urduContent} dir="rtl">
            <div dangerouslySetInnerHTML={{ __html: URDU_CONTENT.replace(/\n/g, '<br/>') }} />
          </div>
        ) : (
          <div className={styles.englishContent}>
            <div dangerouslySetInnerHTML={{ __html: ENGLISH_CONTENT.replace(/\n/g, '<br/>') }} />
          </div>
        )}
      </div>
    </div>
  );
}

export default function UrduToggle() {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <UrduToggleContent />}
    </BrowserOnly>
  );
}
