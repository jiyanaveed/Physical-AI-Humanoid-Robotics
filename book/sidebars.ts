import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  // Define your book structure manually for predictable navigation
  bookSidebar: [ // <<< Changed the ID for clarity, update navbar too!
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2 Fundamentals',
        description: 'Introduction to ROS 2 concepts, Python agents, and URDF.',
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
      items: [
        'module-2-core-ai-algorithms/chapter-2-1-path-planning',
        'module-2-core-ai-algorithms/chapter-2-2-slam-principles',
        'module-2-core-ai-algorithms/chapter-2-3-reinforcement-learning',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception and Vision Fusion',
      items: [
        'module-3-perception-vision-fusion/chapter-3-1-robotics-sensors',
        'module-3-perception-vision-fusion/chapter-3-2-computer-vision',
        'module-3-perception-vision-fusion/chapter-3-3-sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Control and Dynamics',
      items: [
        'module-04/chapter-4-1-kinematics',
        'module-04/chapter-4-2-control-theory',
        'module-04/chapter-4-3-advanced-dynamics',
      ],
    },
  ],
};

export default sidebars;