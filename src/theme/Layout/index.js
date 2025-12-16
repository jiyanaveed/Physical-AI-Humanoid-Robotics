import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <Chatbot />
      </OriginalLayout>
    </>
  );
}