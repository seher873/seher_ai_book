import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '../components/Auth/AuthProvider';

import { ChatbotProvider } from '../components/Chatbot/ChatbotContext';
import ChatbotWrapper from '../components/Chatbot/ChatbotWrapper';

function Layout(props) {
  return (
    <AuthProvider>
      <ChatbotProvider>
        <OriginalLayout {...props}>
          {props.children}

          <BrowserOnly>
            {() => <ChatbotWrapper />}
          </BrowserOnly>

        </OriginalLayout>
      </ChatbotProvider>
    </AuthProvider>
  );
}

export default Layout;
