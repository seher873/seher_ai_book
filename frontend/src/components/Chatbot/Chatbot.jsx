import React, { useState, useRef, useEffect } from 'react';
import PropTypes from 'prop-types';
import { useBaseUrl } from '@docusaurus/useBaseUrl';

import './Chatbot.css';

/**
 * Chatbot Component
 * Implements a chat interface for interacting with the Physical AI textbook RAG system
 */
const Chatbot = ({ isOpen, onClose, onToggle }) => {
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
      // Call backend API to get response via proxy
      const response = await fetch(`/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          max_results: 5
        })
      });

      if (!response.ok) {
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
      const errorMsg = error?.message || String(error) || 'Unknown error';
      console.error('Error getting chat response:', errorMsg);

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