import React from 'react';
import OriginalLayout from '@theme-original/Layout';

// Minimal layout to avoid React not defined error during SSR
function Layout(props) {
  return (
    <OriginalLayout {...props}>
      {props.children}
    </OriginalLayout>
  );
}

export default Layout;
