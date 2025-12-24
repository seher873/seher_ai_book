import React from 'react';
import { useChatbot } from './ChatbotContext';
import Chatbot from './Chatbot';
import ChatbotToggle from './ChatbotToggle';

const ChatbotWrapper = () => {
  const { isChatbotOpen, closeChatbot, toggleChatbot } = useChatbot();

  return (
    <>
      <Chatbot 
        isOpen={isChatbotOpen} 
        onClose={closeChatbot} 
        onToggle={toggleChatbot} 
      />
      {!isChatbotOpen && <ChatbotToggle />}
    </>
  );
};

export default ChatbotWrapper;