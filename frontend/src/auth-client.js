// src/auth-client.js - Custom authentication client for FastAPI backend
import { createContext, useContext, useState, useEffect } from 'react';

// Create auth context
const AuthContext = createContext();

// Custom auth client implementation
class CustomAuthClient {
  constructor() {
    this.baseURL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
  }

  // Signup function
  async signup(userData) {
    const response = await fetch(`${this.baseURL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signup failed');
    }

    const data = await response.json();
    
    // Store token in localStorage
    if (data.access_token) {
      localStorage.setItem('access_token', data.access_token);
    }
    
    return data;
  }

  // Signin function
  async signin(credentials) {
    const response = await fetch(`${this.baseURL}/auth/signin`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(credentials),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signin failed');
    }

    const data = await response.json();
    
    // Store token in localStorage
    if (data.access_token) {
      localStorage.setItem('access_token', data.access_token);
    }
    
    return data;
  }

  // Get current user from token
  async getCurrentUser() {
    const token = localStorage.getItem('access_token');
    
    if (!token) {
      return null;
    }

    try {
      // In a real implementation, you might have a /auth/me endpoint
      // For now, we'll just decode the token to get user info
      const tokenPayload = this.parseJWT(token);
      return {
        id: tokenPayload.sub,
        email: tokenPayload.sub,
        // Add other user properties as needed
      };
    } catch (error) {
      console.error('Error getting current user:', error);
      return null;
    }
  }

  // Signout function
  signout() {
    localStorage.removeItem('access_token');
  }

  // Check if user is authenticated
  isAuthenticated() {
    const token = localStorage.getItem('access_token');
    if (!token) return false;

    try {
      const tokenPayload = this.parseJWT(token);
      const currentTime = Date.now() / 1000;
      return tokenPayload.exp > currentTime;
    } catch (error) {
      return false;
    }
  }

  // Parse JWT token
  parseJWT(token) {
    try {
      const base64Url = token.split('.')[1];
      const base64 = base64Url.replace(/-/g, '+').replace(/_/g, '/');
      const jsonPayload = decodeURIComponent(
        atob(base64)
          .split('')
          .map(c => '%' + ('00' + c.charCodeAt(0).toString(16)).slice(-2))
          .join('')
      );

      return JSON.parse(jsonPayload);
    } catch (error) {
      throw new Error('Invalid token');
    }
  }

  // Get auth token
  getAuthToken() {
    return localStorage.getItem('access_token');
  }
}

const customAuthClient = new CustomAuthClient();

// Auth provider component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is authenticated on initial load
    const checkAuthStatus = async () => {
      try {
        const currentUser = await customAuthClient.getCurrentUser();
        setUser(currentUser);
      } catch (error) {
        console.error('Error checking auth status:', error);
        setUser(null);
      } finally {
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const signup = async (userData) => {
    const result = await customAuthClient.signup(userData);
    setUser(result.user);
    return result;
  };

  const signin = async (credentials) => {
    const result = await customAuthClient.signin(credentials);
    setUser(result.user);
    return result;
  };

  const signout = () => {
    customAuthClient.signout();
    setUser(null);
  };

  const value = {
    user,
    loading,
    isAuthenticated: customAuthClient.isAuthenticated(),
    signup,
    signin,
    signout,
    getAuthToken: customAuthClient.getAuthToken,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

// Export the client for direct use if needed
export { customAuthClient };