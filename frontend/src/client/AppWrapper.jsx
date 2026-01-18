import React from 'react';
import { AuthProvider } from '../components/Auth/AuthProvider';
import { ChatbotProvider } from '../components/Chatbot/ChatbotContext';

// Wrapper component to provide global context to the entire app
export default function AppWrapper({ children }) {
  return (
    <AuthProvider>
      <ChatbotProvider>
        {children}
      </ChatbotProvider>
    </AuthProvider>
  );
}