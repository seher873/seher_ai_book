import React, { useState, useRef, useEffect } from 'react';
import PropTypes from 'prop-types';
import { useBaseUrl } from '@docusaurus/useBaseUrl';
import { useAuth } from '../../auth-client';

import './Chatbot.css';

/**
 * Chatbot Component
 * Implements a chat interface for interacting with the Physical AI textbook RAG system
 */
const Chatbot = ({ isOpen, onClose, onToggle }) => {
  const { isAuthenticated, getAuthToken } = useAuth();
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Physical AI & Humanoid Robotics textbook assistant. Ask me anything about the textbook content!",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Check if user is authenticated
    if (!isAuthenticated) {
      alert('Please sign in to use the chatbot');
      return;
    }

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get auth token
      const token = getAuthToken();

      // Call backend API to get response
      const response = await fetch(`${process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000'}/api/chat`, {  // Changed from /chat to /api/chat
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`  // Add auth token to request
        },
        body: JSON.stringify({
          query: inputValue,
          max_results: 5
        })
      });

      if (!response.ok) {
        if (response.status === 401) {
          alert('Authentication required. Please sign in again.');
          return;
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        timestamp: new Date(),
        sources: data.sources || []
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error getting chat response:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  // Show authentication prompt if not authenticated
  if (!isAuthenticated) {
    return (
      <div className="chatbot-container">
        <div className="chatbot-header">
          <h3>Physical AI Assistant</h3>
          <button className="chatbot-close-btn" onClick={onClose} aria-label="Close chat">
            ×
          </button>
        </div>

        <div className="chatbot-auth-prompt">
          <p>Please sign in to access the chatbot and ask questions about the textbook content.</p>
          <div className="auth-buttons">
            <button
              className="auth-btn signin-btn"
              onClick={() => document.getElementById('signin-modal')?.click()}
            >
              Sign In
            </button>
            <button
              className="auth-btn signup-btn"
              onClick={() => document.getElementById('signup-modal')?.click()}
            >
              Sign Up
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Physical AI Assistant</h3>
        <button className="chatbot-close-btn" onClick={onClose} aria-label="Close chat">
          ×
        </button>
      </div>

      <div className="chatbot-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`chatbot-message ${message.sender}-message`}
          >
            <div className="message-content">
              <p>{message.text}</p>

              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <small>Sources:</small>
                  <ul>
                    {message.sources.slice(0, 3).map((source, idx) => (
                      <li key={idx}>
                        {source.title || source.path || 'Source'}
                        {source.score && ` (Relevance: ${(source.score * 100).toFixed(1)}%)`}
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
            <div className="message-timestamp">
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="chatbot-message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chatbot-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask about Physical AI concepts..."
          disabled={isLoading}
          className="chatbot-input"
        />
        <button
          type="submit"
          disabled={!inputValue.trim() || isLoading}
          className="chatbot-send-btn"
        >
          Send
        </button>
      </form>
    </div>
  );
};

Chatbot.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  onToggle: PropTypes.func.isRequired
};

export default Chatbot;