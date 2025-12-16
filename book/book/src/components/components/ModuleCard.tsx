// components/ModuleCard.tsx
import Link from 'next/link';

interface ModuleCardProps {
  title: string;
  items: string[];
  href: string; // URL for the module
}

export function ModuleCard({ title, items, href }: ModuleCardProps) {
  return (
    <Link href={href} className="module-card">
      <h3 className="card-title">{title}</h3>
      <ul className="card-item-list">
        {items.map((item, index) => (
          <li key={index}>{item}</li>
        ))}
      </ul>
    </Link>
  );
}
