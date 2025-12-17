import React, { useState, useEffect } from 'react';
import './RagChatbot.css';

/**
 * RagChatbot Component
 * Main chatbot interface for the RAG chatbot feature
 */
const RagChatbot = ({ textbookContent }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);

  // Initialize a new session when the component mounts
  useEffect(() => {
    const initializeSession = async () => {
      try {
        const response = await fetch('/api/v1/sessions', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
        });
        const data = await response.json();
        setSessionId(data.session_token);
      } catch (error) {
        console.error('Error initializing session:', error);
      }
    };

    initializeSession();
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || !sessionId) return;

    // Add user message to the chat
    const userMessage = { id: Date.now(), text: inputValue, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Determine if this is a global query or focused on selected text
      const requestBody = {
        session_token: sessionId,
        message: inputValue,  // Updated from 'query' to match API
        query_type: 'global', // or 'focused' if user has selected text
        selected_text: null   // Would include selected text if focused query
        // Note: We're not currently sending conversation history from frontend
        // The backend maintains conversation history in the session
      };

      const response = await fetch('/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

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
    } catch (error) {
      console.error('Error getting response from chatbot:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'ai',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleCitationClick = (chapterNumber, sectionNumber) => {
    // Navigate to the cited section in the textbook
    // This will likely involve updating the textbook viewer component or opening in a new tab
    const citationUrl = `/textbook/chapters/${encodeURIComponent(chapterNumber)}/sections/${encodeURIComponent(sectionNumber)}`;

    // For a Docusaurus-based textbook, we might want to navigate to the specific section
    // This could be done by:
    // 1. Using the router if we're in a React app
    // 2. Using window.location for simple navigation
    // 3. Passing an onNavigate callback prop to handle the navigation in the parent component

    // For now, we'll use window.location to navigate to the section
    window.location.href = citationUrl;
  };

  const renderCitations = (citations) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="citations">
        <p><strong>Citations:</strong></p>
        <ul>
          {citations.map((citation, index) => (
            <li key={index}>
              <button
                onClick={() => handleCitationClick(citation.chapter_number, citation.section_number)}
                className="citation-link"
              >
                Section {citation.chapter_number}.{citation.section_number}: {citation.text_excerpt.substring(0, 50)}...
              </button>
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Physical AI Textbook Assistant</h3>
      </div>
      
      <div className="chat-messages">
        {messages.map((message) => (
          <div key={message.id} className={`message ${message.sender}`}>
            <div className="message-content">
              <p>{message.text}</p>
              {message.sender === 'ai' && renderCitations(message.citations)}
            </div>
          </div>
        ))}
        
        {isLoading && (
          <div className="message ai">
            <div className="message-content">
              <p>Thinking...</p>
            </div>
          </div>
        )}
      </div>
      
      <form onSubmit={handleSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the Physical AI textbook..."
          disabled={isLoading || !sessionId}
        />
        <button type="submit" disabled={isLoading || !sessionId}>
          Send
        </button>
      </form>
    </div>
  );
};

export default RagChatbot;