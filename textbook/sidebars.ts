import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Front Matter',
      collapsed: false,
      items: [
        'preface',
        'how-to-use',
        'course-mapping',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module1/intro',
        'module1/chapter1-1-ros2-fundamentals',
        'module1/chapter1-2-nodes-communication',
        'module1/chapter1-3-launch-files',
        'module1/chapter1-4-packages',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module2/intro',
        'module2/chapter2-1-digital-twin-intro',
        'module2/chapter2-2-gazebo-fundamentals',
        'module2/chapter2-3-unity-robotics',
        'module2/chapter2-4-sensors-vslam',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        'module3/intro',
        'module3/chapter3-1-isaac-overview',
        'module3/chapter3-2-isaac-perception',
        'module3/chapter3-3-isaac-manipulation-nav',
        'module3/chapter3-4-isaac-gym-rl',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'module4/intro',
        'module4/chapter4-1-vla-intro',
        'module4/chapter4-2-llm-integration',
        'module4/chapter4-3-whisper-voice',
        'module4/chapter4-4-vla-system',
      ],
    },
    {
      type: 'category',
      label: 'Back Matter',
      collapsed: false,
      items: [
        'glossary',
        'references',
        {
          type: 'category',
          label: 'Appendices',
          items: [
            'appendices/hardware-setup',
            'appendices/software-installation',
            'appendices/launch-files',
            'appendices/sim2real',
            'appendices/cloud-vs-onprem',
            'appendices/troubleshooting',
            'appendices/resources',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
