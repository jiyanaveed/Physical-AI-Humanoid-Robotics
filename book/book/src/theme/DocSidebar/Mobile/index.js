// /Users/javerianaveed/myEbook/book/src/theme/DocSidebar/Mobile/index.js
import React from 'react';
import DocSidebarMobile from '@theme-original/DocSidebar/Mobile'; // Import original
import { useLocation } from '@docusaurus/router';

export default function DocSidebarMobileWrapper(props) {
  const location = useLocation();
  const isHomepage = location.pathname === '/';

  if (isHomepage) {
    return null; // Hide sidebar on homepage
  }

  return <DocSidebarMobile {...props} />; // Render original sidebar otherwise
}
