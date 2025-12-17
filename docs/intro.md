---
id: intro
sidebar_position: 1
hide_sidebar: true
hide_title: true
---

import { ModuleCard } from '@site/src/components/ModuleCard';
import Link from '@docusaurus/Link';

<div className="homepage-container">
  <section className="hero-section">
    <h1 className="hero-title">WELCOME TO THE BOOK OF PHYSICAL AI AND HUMANOIDS</h1>
    <p className="hero-description">Master the Dynamics, Control, and Intelligence of Bipedal Systems.</p>
    <Link to="/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services" className="start-reading-button">
      Start Reading Now!
    </Link>
    <p className="intro-text">
      This documentation serves as the comprehensive, code-driven guide for learning advanced concepts in robotics, control theory, and artificial intelligence, with a focus on building complex humanoid systems using ROS 2.
    </p>
  </section>

  <section className="modules-section">
    <h2 className="section-title">📚 Book Structure: Four Pillars of Robotics</h2>
    <p className="section-description">
      The book is organized into four distinct modules, each focusing on a core area of the humanoid robotics stack.
    </p>
    <div className="modules-grid">
      <ModuleCard
        title="Module 1: ROS 2 Fundamentals"
        href="/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services"
        items={['ROS 2 workspace setup', 'Nodes, topics, services', 'Initial URDF for humanoid kinematics']}
      />
      <ModuleCard
        title="Module 2: Core AI Algorithms"
        href="/module-2-core-ai-algorithms/chapter-2-1-path-planning"
        items={['Path Planning (A*, Dijkstra)', 'SLAM', 'Reinforcement Learning in simulation']}
      />
      <ModuleCard
        title="Module 3: Control and Planning"
        href="/module-3-control-planning/chapter-3-1-robotics-sensors"
        items={['Robotics Sensors', 'Advanced Computer Vision', 'Sensor Fusion']}
      />
      <ModuleCard
        title="Module 4: Advanced Kinematics & Dynamics"
        href="/module-04/chapter-4-1-kinematics"
        items={['Homogeneous Transformations', 'Forward & Inverse Kinematics', 'PID Control', 'Robot Dynamics (Lagrangian)', 'Non-linear Control (ZMP, Computed Torque)']}
      />
    </div>
  </section>

  <section className="get-started-section">
    <h2 className="section-title">🚀 GET STARTED</h2>
    <ol className="get-started-list">
      <li>Start with the basics: Click on any module card above to dive in.</li>
      <li>Explore the code: Visit the <a href="https://github.com/jiyanaveed/Physical-AI-Humanoid-Robotics" target="_blank" rel="noopener noreferrer">GitHub Repository</a> to clone the examples and follow along.</li>
    </ol>
  </section>
</div>