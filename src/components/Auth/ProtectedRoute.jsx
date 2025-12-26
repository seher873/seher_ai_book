import React from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import { useAuth } from '../../auth-client';

const ProtectedRoute = ({ children }) => {
  const { session, isPending } = useAuth();
  const location = useLocation();

  if (isPending) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        height: '100vh',
        fontSize: '18px'
      }}>
        Loading...
      </div>
    );
  }

  if (!session) {
    // Redirect to login page with return url
    return <Navigate to={`/auth/login?return=${location.pathname}`} replace />;
  }

  return children;
};

export default ProtectedRoute;