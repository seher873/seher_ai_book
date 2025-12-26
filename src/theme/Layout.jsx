import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { authClient } from '../auth-client';

import { ChatbotProvider } from '../components/Chatbot/ChatbotContext';
import ChatbotWrapper from '../components/Chatbot/ChatbotWrapper';

function Layout(props) {
  return (
    <authClient.Provider>
      <ChatbotProvider>
        <OriginalLayout {...props}>
          {props.children}

          <BrowserOnly>
            {() => <ChatbotWrapper />}
          </BrowserOnly>

        </OriginalLayout>
      </ChatbotProvider>
    </authClient.Provider>
  );
}

export default Layout;
