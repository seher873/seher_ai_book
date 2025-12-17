import { useState, useEffect } from 'react';

/**
 * Custom hook for managing RAG chat functionality
 * Handles session management, message history, and API communication
 */
const useRagChat = () => {
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Initialize a new session when the hook is used
  useEffect(() => {
    const initializeSession = async () => {
      try {
        setIsLoading(true);
        const response = await fetch('/api/v1/sessions', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (!response.ok) {
          throw new Error(`Session initialization failed: ${response.status}`);
        }

        const data = await response.json();
        setSessionId(data.session_token);
        setError(null);
      } catch (err) {
        setError(`Failed to initialize session: ${err.message}`);
        console.error('Session initialization error:', err);
      } finally {
        setIsLoading(false);
      }
    };

    initializeSession();
  }, []);

  const sendMessage = async (query, queryContext = { type: 'global', selected_text: null }) => {
    if (!query.trim() || !sessionId) {
      return Promise.reject(new Error('Invalid query or session'));
    }

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text: query,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const requestBody = {
        session_token: sessionId,
        message: query,  // Updated to match backend API
        query_type: queryContext.type || 'global',  // Updated to match backend API
        selected_text: queryContext.selected_text || null  // Updated to match backend API
      };

      const response = await fetch('/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error?.message || `Chat request failed: ${response.status}`);
      }

      const data = await response.json();

      // Add AI response to the chat
      const aiMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'ai',
        citations: data.citations,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, aiMessage]);
      return aiMessage;
    } catch (err) {
      setError(`Failed to get response: ${err.message}`);
      console.error('Chat error:', err);

      // Add error message to the chat
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'ai',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  const getCurrentSessionId = () => sessionId;

  return {
    messages,
    sendMessage,
    clearChat,
    isLoading,
    error,
    sessionId: getCurrentSessionId(),
  };
};

export default useRagChat;