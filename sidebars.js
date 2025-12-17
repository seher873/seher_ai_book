/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'resume',
    'course-overview',
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI and ROS2 Framework',
      items: [
        'ch1-introduction/sub1-what-is-physical-ai',
        'ch1-introduction/sub2-ros2-overview',
        'ch1-introduction/sub3-setup-env',
        'ch1-introduction/sub4-architecture-elements',
        'ch1-introduction/sub5-packages-workspaces',
        'ch1-introduction/sub6-commands-tools'
      ],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        'modules/mod1-ros2-deep-dive',
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'modules/mod2-digital-twin-gazebo-unity',
            'modules/ch2-1-intro-digital-twins',
            'modules/ch2-2-gazebo-fundamentals',
            'modules/ch2-3-unity-robotics-simulation',
            'modules/ch2-4-ros2-integration-simulation',
            'modules/ch2-5-hands-on-labs-troubleshooting'
          ],
        },
        {
          type: 'category',
          label: 'Module 3: Simulation and Training Pipelines',
          items: [
            'module-3/index',
            'module-3/intro-isaac',
            'module-3/isaac-sim-setup',
            'module-3/synthetic-data',
            'module-3/isaac-ros',
            'module-3/hands-on-labs'
          ],
        },
        {
          type: 'category',
          label: 'Module 4: LLMs + Robotics - Voice-to-Action Systems',
          items: [
            'module-4/index',
            'module-4/intro-vla',
            'module-4/speech-to-text',
            'module-4/task-decomposition',
            'module-4/multimodal-perception',
            'module-4/ros2-planning-execution',
            'module-4/capstone-humanoid'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Labs',
      items: [
        'labs/lab1-1-ros2-installation',
        'labs/lab1-2-first-nodes'
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/module-1/introduction-quiz',
      ],
    },
    {
      type: 'link',
      label: 'AI Physics Playground',
      href: '/docs/physics-playground',
    },
    {
      type: 'link',
      label: 'Lab Exercises',
      href: '/docs/labs',
    },
    {
      type: 'link',
      label: 'Hardware Guidelines',
      href: '/docs/hardware-guidelines',
    },
    {
      type: 'category',
      label: 'Glossary',
      items: [
        'glossary/index',
        'glossary/a-e',
        'glossary/f-j',
        'glossary/k-o',
        'glossary/p-t',
        'glossary/u-z',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/index',
        'appendices/appendix-b-chatbot-rag',
      ],
    },
  ],
};

module.exports = sidebars;