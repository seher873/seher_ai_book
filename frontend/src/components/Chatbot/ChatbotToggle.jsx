import React from 'react';
import { useChatbot } from './ChatbotContext';
import './Chatbot.css';

const ChatbotToggle = () => {
  const { isChatbotOpen, toggleChatbot } = useChatbot();

  if (isChatbotOpen) return null; // Don't show toggle when chat is open

  return (
    <button 
      className="chatbot-toggle-btn" 
      onClick={toggleChatbot}
      aria-label="Open chatbot"
    >
      💬
    </button>
  );
};

export default ChatbotToggle;