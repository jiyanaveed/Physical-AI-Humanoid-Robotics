import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface ModuleItem {
  title: string;
  icon: string; // e.g., '🤖', '🧠', '⚙️'
  description: JSX.Element;
  link: string;
}

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: ROS 2 & Fundamentals',
    icon: '🚀',
    description: (
      <>
        Set up your ROS 2 workspace, master nodes, topics, services, and build initial URDF models for humanoid kinematics.
      </>
    ),
    link: '/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services',
  },
  {
    title: 'Module 2: Core AI Algorithms',
    icon: '🧠',
    description: (
      <>
        Dive into Path Planning (A*, Dijkstra), SLAM, and foundational Reinforcement Learning in simulation environments.
      </>
    ),
    link: '/module-2-core-ai-algorithms/chapter-2-1-path-planning',
  },
  {
    title: 'Module 3: Control and Planning',
    icon: '⚙️',
    description: (
      <>
        Explore Robotics Sensors, Advanced Computer Vision for perception, and Sensor Fusion techniques for robust state estimation.
      </>
    ),
    link: '/module-3-control-planning/chapter-3-1-robotics-sensors',
  },
  {
    title: 'Module 4: Advanced Kinematics & Dynamics',
    icon: '🤖',
    description: (
      <>
        Master Homogeneous Transformations, Forward/Inverse Kinematics, PID Control, Robot Dynamics (Lagrangian), and Non-Linear Control (ZMP, Computed Torque).
      </>
    ),
    link: '/module-04/chapter-4-1-kinematics',
  },
];

function Module({ title, icon, description, link }: ModuleItem) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <div className="card shadow--md">
        <div className="card__header">
          <div className={styles.moduleIcon}>{icon}</div>
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link
            className="button button--secondary button--block"
            to={link}>
            Learn More
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function ModuleCards(): JSX.Element {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}