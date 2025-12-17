import React from 'react';
import Layout from '@theme/Layout';

const Homepage = () => {
  return (
    <Layout title="Home" description="Physical AI and Humanoid Robotics Book">
      <div className="homepage">
        <section className="hero-banner">
          <div className="hero-content">
            <div className="robotic-visual">
              <div className="robotic-dot"></div>
              <div className="robotic-dot"></div>
              <div className="robotic-dot"></div>
              <div className="robotic-dot"></div>
              <div className="robotic-dot"></div>
            </div>
            <h1 className="hero-title">WELCOME TO THE BOOK OF PHYSICAL AI AND HUMANOIDS</h1>
            <p className="hero-subtitle">Master the Dynamics, Control, and Intelligence of Bipedal Systems.</p>
            <a href="/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services" className="start-reading-button">
              Start Reading Now!
            </a>
            <p className="hero-description">
              This documentation serves as the comprehensive, code-driven guide for learning advanced concepts in robotics, control theory, and artificial intelligence, with a focus on building complex humanoid systems using ROS 2.
            </p>
          </div>
        </section>

        <section className="modules-section">
          <h2 className="section-title">📚 Book Structure: Four Pillars of Robotics</h2>
          <p className="section-description">
            The book is organized into four distinct modules, each focusing on a core area of the humanoid robotics stack.
          </p>
          <div className="modules-grid">
            <div className="module-card">
              <h3>Module 1: ROS 2 Fundamentals</h3>
              <ul>
                <li>ROS 2 workspace setup</li>
                <li>Nodes, topics, services</li>
                <li>Initial URDF for humanoid kinematics</li>
              </ul>
              <a href="/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services" className="explore-module-button">
                Explore Module
              </a>
            </div>
            <div className="module-card">
              <h3>Module 2: Core AI Algorithms</h3>
              <ul>
                <li>Path Planning (A*, Dijkstra)</li>
                <li>SLAM</li>
                <li>Reinforcement Learning in simulation</li>
              </ul>
              <a href="/module-2-core-ai-algorithms/chapter-2-1-path-planning" className="explore-module-button">
                Explore Module
              </a>
            </div>
            <div className="module-card">
              <h3>Module 3: Control and Planning</h3>
              <ul>
                <li>Robotics Sensors</li>
                <li>Advanced Computer Vision</li>
                <li>Sensor Fusion</li>
              </ul>
              <a href="/module-3-control-planning/chapter-3-1-robotics-sensors" className="explore-module-button">
                Explore Module
              </a>
            </div>
            <div className="module-card">
              <h3>Module 4: Advanced Kinematics & Dynamics</h3>
              <ul>
                <li>Homogeneous Transformations</li>
                <li>Forward & Inverse Kinematics</li>
                <li>PID Control</li>
                <li>Robot Dynamics (Lagrangian)</li>
                <li>Non-linear Control (ZMP, Computed Torque)</li>
              </ul>
              <a href="/module-04/chapter-4-1-kinematics" className="explore-module-button">
                Explore Module
              </a>
            </div>
          </div>
        </section>

      </div>
    </Layout>
  );
};

export default Homepage;