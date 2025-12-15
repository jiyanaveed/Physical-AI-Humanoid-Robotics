import React, { useEffect, useRef } from 'react';
import katex from 'katex';
import 'katex/dist/katex.min.css';

interface LatexProps {
  children: string;
  macros?: object;
}

const Latex: React.FC<LatexProps> = ({ children, macros }) => {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (containerRef.current) {
      const content = React.Children.toArray(children).join('').trim();
      const displayMode = content.startsWith('$$') && content.endsWith('$$');
      const latex = displayMode
        ? content.substring(2, content.length - 2).trim()
        : content.substring(1, content.length - 1).trim();
        
      katex.render(latex, containerRef.current, {
        throwOnError: false,
        displayMode,
        macros,
      });
    }
  }, [children, macros]);

  return <div ref={containerRef} />;
};

export default Latex;
