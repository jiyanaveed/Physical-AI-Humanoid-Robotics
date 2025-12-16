import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services',
      },
      items: [
        'module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services',
        'module-1-ros2-fundamentals/chapter-1-2-python-agents-ros',
        'module-1-ros2-fundamentals/chapter-1-3-urdf-humanoid-kinematics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Core AI Algorithms',
      link: {
        type: 'doc',
        id: 'module-2-core-ai-algorithms/chapter-2-1-path-planning',
      },
      items: [
        'module-2-core-ai-algorithms/chapter-2-1-path-planning',
        'module-2-core-ai-algorithms/chapter-2-2-slam',
        'module-2-core-ai-algorithms/chapter-2-3-rl-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Control and Planning',
      link: {
        type: 'doc',
        id: 'module-3-control-planning/chapter-3-1-robotics-sensors',
      },
      items: [
        'module-3-control-planning/chapter-3-1-robotics-sensors',
        'module-3-control-planning/chapter-3-2-computer-vision',
        'module-3-control-planning/chapter-3-3-sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Advanced Kinematics and Dynamics',
      link: {
        type: 'doc',
        id: 'module-04/chapter-4-1-kinematics',
      },
      items: [
        'module-04/chapter-4-1-kinematics',
        'module-04/chapter-4-2-control-theory',
        'module-04/chapter-4-3-advanced-dynamics',
      ],
    },
  ],
};

export default sidebars;
