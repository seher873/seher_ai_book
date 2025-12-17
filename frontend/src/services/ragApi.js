/**
 * API service for RAG chatbot communication
 * Provides methods to interact with the backend API
 */

// Base API URL - can be configured via environment variables
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || '/api/v1';

/**
 * Initialize a new chat session
 * @returns {Promise<Object>} Session data with session_token
 */
export const createSession = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/sessions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Session creation failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error creating session:', error);
    throw error;
  }
};

/**
 * Get session details
 * @param {string} sessionToken - The session token
 * @returns {Promise<Object>} Session details
 */
export const getSession = async (sessionToken) => {
  try {
    const response = await fetch(`${API_BASE_URL}/sessions/${sessionToken}`);

    if (!response.ok) {
      throw new Error(`Session fetch failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching session:', error);
    throw error;
  }
};

/**
 * Send a message to the chatbot
 * @param {string} sessionToken - The session token
 * @param {string} query - The user's query
 * @param {Object} queryContext - Context for the query (type, selected_text, etc.)
 * @returns {Promise<Object>} Response from the chatbot
 */
export const sendMessage = async (sessionToken, query, queryContext = { type: 'global' }) => {
  try {
    const response = await fetch(`${API_BASE_URL}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        session_token: sessionToken,
        query: query,
        query_context: queryContext
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.error?.message || `Chat request failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error sending message:', error);
    throw error;
  }
};

/**
 * Validate content using the moderation API
 * @param {string} content - The content to validate
 * @param {string} contentType - Type of content ('query' or 'response')
 * @returns {Promise<Object>} Validation result
 */
export const validateContent = async (content, contentType = 'query') => {
  try {
    const response = await fetch(`${API_BASE_URL}/validate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        content: content,
        content_type: contentType
      }),
    });

    if (!response.ok) {
      throw new Error(`Content validation failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error validating content:', error);
    throw error;
  }
};

/**
 * Get list of textbook chapters
 * @returns {Promise<Array>} List of chapters with sections
 */
export const getTextbookChapters = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/textbook/chapters`);

    if (!response.ok) {
      throw new Error(`Fetching textbook chapters failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching textbook chapters:', error);
    throw error;
  }
};

/**
 * Get content of a specific textbook section
 * @param {string} chapterNumber - The chapter number
 * @param {string} sectionNumber - The section number
 * @returns {Promise<Object>} Section content
 */
export const getTextbookSection = async (chapterNumber, sectionNumber) => {
  try {
    const response = await fetch(`${API_BASE_URL}/textbook/chapters/${chapterNumber}/sections/${sectionNumber}`);

    if (!response.ok) {
      throw new Error(`Fetching textbook section failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching textbook section:', error);
    throw error;
  }
};