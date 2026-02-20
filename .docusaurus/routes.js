import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical_ai_textbook/markdown-page',
    component: ComponentCreator('/physical_ai_textbook/markdown-page', '3b0'),
    exact: true
  },
  {
    path: '/physical_ai_textbook/docs',
    component: ComponentCreator('/physical_ai_textbook/docs', 'a6e'),
    routes: [
      {
        path: '/physical_ai_textbook/docs',
        component: ComponentCreator('/physical_ai_textbook/docs', '57d'),
        routes: [
          {
            path: '/physical_ai_textbook/docs',
            component: ComponentCreator('/physical_ai_textbook/docs', 'd21'),
            routes: [
              {
                path: '/physical_ai_textbook/docs/appendices/cloud-vs-onprem',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/cloud-vs-onprem', '205'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/appendices/hardware-setup',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/hardware-setup', '778'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/appendices/launch-files',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/launch-files', 'a6f'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/appendices/resources',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/resources', '7c6'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/appendices/software-installation',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/software-installation', '37a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/appendices/troubleshooting',
                component: ComponentCreator('/physical_ai_textbook/docs/appendices/troubleshooting', '877'),
                exact: true
              },
              {
                path: '/physical_ai_textbook/docs/course-mapping',
                component: ComponentCreator('/physical_ai_textbook/docs/course-mapping', '15d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/glossary',
                component: ComponentCreator('/physical_ai_textbook/docs/glossary', '53a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/how-to-use',
                component: ComponentCreator('/physical_ai_textbook/docs/how-to-use', 'f50'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/intro',
                component: ComponentCreator('/physical_ai_textbook/docs/intro', '619'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals',
                component: ComponentCreator('/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals', 'e87'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module1/chapter1-2-nodes-communication',
                component: ComponentCreator('/physical_ai_textbook/docs/module1/chapter1-2-nodes-communication', '41e'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module1/chapter1-3-launch-files',
                component: ComponentCreator('/physical_ai_textbook/docs/module1/chapter1-3-launch-files', 'dd2'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module1/chapter1-4-packages',
                component: ComponentCreator('/physical_ai_textbook/docs/module1/chapter1-4-packages', '321'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module1/intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module1/intro', '25d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module2/chapter2-1-digital-twin-intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module2/chapter2-1-digital-twin-intro', 'ea4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module2/chapter2-2-gazebo-fundamentals',
                component: ComponentCreator('/physical_ai_textbook/docs/module2/chapter2-2-gazebo-fundamentals', '188'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module2/chapter2-3-unity-robotics',
                component: ComponentCreator('/physical_ai_textbook/docs/module2/chapter2-3-unity-robotics', '2d7'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module2/chapter2-4-sensors-vslam',
                component: ComponentCreator('/physical_ai_textbook/docs/module2/chapter2-4-sensors-vslam', 'dac'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module2/intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module2/intro', 'f86'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module3/chapter3-1-isaac-overview',
                component: ComponentCreator('/physical_ai_textbook/docs/module3/chapter3-1-isaac-overview', '1d2'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module3/chapter3-2-isaac-perception',
                component: ComponentCreator('/physical_ai_textbook/docs/module3/chapter3-2-isaac-perception', '21a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module3/chapter3-3-isaac-manipulation-nav',
                component: ComponentCreator('/physical_ai_textbook/docs/module3/chapter3-3-isaac-manipulation-nav', 'f5a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module3/chapter3-4-isaac-gym-rl',
                component: ComponentCreator('/physical_ai_textbook/docs/module3/chapter3-4-isaac-gym-rl', '109'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module3/intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module3/intro', 'be5'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module4/chapter4-1-vla-intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module4/chapter4-1-vla-intro', 'a65'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module4/chapter4-2-llm-integration',
                component: ComponentCreator('/physical_ai_textbook/docs/module4/chapter4-2-llm-integration', 'bb8'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module4/chapter4-3-whisper-voice',
                component: ComponentCreator('/physical_ai_textbook/docs/module4/chapter4-3-whisper-voice', 'b12'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module4/chapter4-4-vla-system',
                component: ComponentCreator('/physical_ai_textbook/docs/module4/chapter4-4-vla-system', '5f6'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/module4/intro',
                component: ComponentCreator('/physical_ai_textbook/docs/module4/intro', 'fff'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/preface',
                component: ComponentCreator('/physical_ai_textbook/docs/preface', '3e0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/references',
                component: ComponentCreator('/physical_ai_textbook/docs/references', 'e08'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical_ai_textbook/docs/week-to-chapter-mapping',
                component: ComponentCreator('/physical_ai_textbook/docs/week-to-chapter-mapping', '450'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical_ai_textbook/',
    component: ComponentCreator('/physical_ai_textbook/', '5e2'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
