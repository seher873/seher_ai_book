import React, { createContext, useContext, useState } from 'react';
import PropTypes from 'prop-types';

const ChatbotContext = createContext();

export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
};

export const ChatbotProvider = ({ children }) => {
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  const openChatbot = () => setIsChatbotOpen(true);
  const closeChatbot = () => setIsChatbotOpen(false);
  const toggleChatbot = () => setIsChatbotOpen(prev => !prev);

  return (
    <ChatbotContext.Provider value={{
      isChatbotOpen,
      openChatbot,
      closeChatbot,
      toggleChatbot
    }}>
      {children}
    </ChatbotContext.Provider>
  );
};

ChatbotProvider.propTypes = {
  children: PropTypes.node.isRequired
};