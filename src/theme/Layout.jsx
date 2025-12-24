import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

import { ChatbotProvider } from '../components/Chatbot/ChatbotContext';
import ChatbotWrapper from '../components/Chatbot/ChatbotWrapper';

function Layout(props) {
  return (
    <ChatbotProvider>
      <OriginalLayout {...props}>
        {props.children}

        <BrowserOnly>
          {() => <ChatbotWrapper />}
        </BrowserOnly>

      </OriginalLayout>
    </ChatbotProvider>
  );
}

export default Layout;
