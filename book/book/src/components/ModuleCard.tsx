import React from 'react';
import Link from '@docusaurus/Link';
import styles from './ModuleCard.module.css';

interface ModuleCardProps {
  title: string;
  href: string;
  items: string[];
}

export const ModuleCard = ({ title, href, items }: ModuleCardProps) => {
  return (
    <div className={styles.moduleCard}>
      <h3>{title}</h3>
      <ul>
        {items.map((item, index) => (
          <li key={index}>{item}</li>
        ))}
      </ul>
      <Link to={href}>
        Explore Module
      </Link>
    </div>
  );
};
