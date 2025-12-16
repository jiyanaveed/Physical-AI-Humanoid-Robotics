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
      // OLD Label: 'Module 2: Core AI Algorithms' 
      label: 'Module 2: Simulation and Sensors', // Corrected to match your folder/plan
      items: [
        // CORRECTED PATH: module-2-simulation-sensors
        'module-2-simulation-sensors/chapter-2-1-path-planning',
        'module-2-simulation-sensors/chapter-2-2-slam-principles',
        'module-2-simulation-sensors/chapter-2-3-reinforcement-learning',
      ],
    },
    {
      type: 'category',
      // OLD Label: 'Module 3: Perception and Vision Fusion'
      label: 'Module 3: Perception and Vision', // Corrected to match your folder/plan
      items: [
        // CORRECTED PATH: module-3-perception-vision
        'module-3-perception-vision/chapter-3-1-robotics-sensors',
        'module-3-perception-vision/chapter-3-2-computer-vision',
        'module-3-perception-vision/chapter-3-3-sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Control and Dynamics',
      items: [
        // CORRECTED PATH: module-4-control-dynamics
        'module-4-control-dynamics/chapter-4-1-kinematics',
        'module-4-control-dynamics/chapter-4-2-control-theory',
        'module-4-control-dynamics/chapter-4-3-advanced-dynamics',
      ],
    },
  ],
};

export default sidebars;