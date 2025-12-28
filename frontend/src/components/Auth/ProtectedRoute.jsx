// src/components/Auth/ProtectedRoute.jsx
import React from 'react';
import { useAuth } from '../../auth-client';
import { Navigate } from 'react-router-dom'; // If using React Router

const ProtectedRoute = ({ children, redirectTo = '/login' }) => {
  const { isAuthenticated, loading } = useAuth();

  // If auth is still loading, show a loading state
  if (loading) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
        <p>Loading...</p>
      </div>
    );
  }

  // If user is not authenticated, redirect to login page
  if (!isAuthenticated) {
    // For Docusaurus context, we might want to show a modal or message instead of redirecting
    return (
      <div style={{ padding: '20px', textAlign: 'center' }}>
        <h2>Authentication Required</h2>
        <p>Please sign in to access this content.</p>
        <button 
          onClick={() => document.getElementById('auth-modal')?.click()}
          style={{
            padding: '10px 20px',
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer'
          }}
        >
          Sign In
        </button>
      </div>
    );
  }

  // If authenticated, render the children
  return children;
};

export default ProtectedRoute;