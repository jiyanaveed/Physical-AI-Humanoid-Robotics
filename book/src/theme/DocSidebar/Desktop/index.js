// /Users/javerianaveed/myEbook/book/src/theme/DocSidebar/Desktop/index.js
import React from 'react';
import DocSidebarDesktop from '@theme-original/DocSidebar/Desktop'; // Import original
import { useLocation } from '@docusaurus/router';

export default function DocSidebarDesktopWrapper(props) {
  const location = useLocation();
  const isHomepage = location.pathname === '/';

  if (isHomepage) {
    return null; // Hide sidebar on homepage
  }

  return <DocSidebarDesktop {...props} />; // Render original sidebar otherwise
}
