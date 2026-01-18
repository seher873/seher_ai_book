import React from 'react';
import { useAuth } from './AuthProvider'; // Using consistent auth system

const ProtectedRoute = ({ children }) => {
  const { user, loading } = useAuth(); // Using the custom auth context

  if (loading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '70vh',
        fontSize: '18px'
      }}>
        Loading...
      </div>
    );
  }

  if (!user) {
    // In Docusaurus, we use window.location for navigation instead of React Router
    if (typeof window !== 'undefined') {
      const returnUrl = window.location.pathname;
      window.location.href = `/auth/login?return=${encodeURIComponent(returnUrl)}`;
    }
    return <div>Redirecting...</div>;
  }

  return children;
};

export default ProtectedRoute;