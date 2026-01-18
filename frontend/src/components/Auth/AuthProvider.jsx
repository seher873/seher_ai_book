// src/components/Auth/AuthProvider.jsx
import React, { createContext, useContext, useEffect, useState } from 'react';
import axios from 'axios';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  // Check if user is logged in on initial load
  useEffect(() => {
    // Check if we're in the browser environment before accessing localStorage
    if (typeof window !== 'undefined') {
      const token = localStorage.getItem('token');
      if (token) {
        // Verify token and get user info
        axios.defaults.headers.common['Authorization'] = `Bearer ${token}`;
        fetchUser();
      } else {
        setLoading(false);
      }
    } else {
      // On the server, just set loading to false
      setLoading(false);
    }
  }, []);

  const fetchUser = async () => {
    try {
      const response = await axios.get('/api/auth/me');
      setUser(response.data);
    } catch (err) {
      console.error('Error fetching user:', err?.message || String(err));
      // Only access localStorage in browser environment
      if (typeof window !== 'undefined') {
        localStorage.removeItem('token');
        delete axios.defaults.headers.common['Authorization'];
      }
    } finally {
      setLoading(false);
    }
  };

  const signup = async (email, password, name, softwareBackground, hardwareBackground) => {
    try {
      setError(null);
      const response = await axios.post('/api/auth/signup', {
        email,
        password,
        name,
        software_background: softwareBackground,
        hardware_background: hardwareBackground
      });

      const { access_token } = response.data;
      // Only access localStorage in browser environment
      if (typeof window !== 'undefined') {
        localStorage.setItem('token', access_token);
        axios.defaults.headers.common['Authorization'] = `Bearer ${access_token}`;
      }

      setUser(response.data.user);
      return response.data;
    } catch (err) {
      const errorMsg = err.response?.data?.detail || err?.message || 'Signup failed';
      console.error('Signup error:', errorMsg);
      setError(errorMsg);
      throw err;
    }
  };

  const signin = async (email, password) => {
    try {
      setError(null);
      const response = await axios.post('/api/auth/signin', {
        email,
        password
      });

      const { access_token } = response.data;
      // Only access localStorage in browser environment
      if (typeof window !== 'undefined') {
        localStorage.setItem('token', access_token);
        axios.defaults.headers.common['Authorization'] = `Bearer ${access_token}`;
      }

      setUser(response.data.user);
      return response.data;
    } catch (err) {
      const errorMsg = err.response?.data?.detail || err?.message || 'Signin failed';
      console.error('Signin error:', errorMsg);
      setError(errorMsg);
      throw err;
    }
  };

  const signout = () => {
    // Only access localStorage in browser environment
    if (typeof window !== 'undefined') {
      localStorage.removeItem('token');
      delete axios.defaults.headers.common['Authorization'];
    }
    setUser(null);
  };

  const value = {
    user,
    loading,
    error,
    signup,
    signin,
    signout,
    isAuthenticated: !!user
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};