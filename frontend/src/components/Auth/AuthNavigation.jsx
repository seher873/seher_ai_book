import React from 'react';
import { useAuth } from '../../auth-client';
import Link from '@docusaurus/Link';

const AuthNavigation = () => {
  const { user, loading, signout, isAuthenticated } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="auth-nav">
      {isAuthenticated ? (
        <div className="user-menu">
          <span>Welcome, {user?.name || user?.email}!</span>
          <button
            onClick={() => {
              signout();
              // Refresh the page to update UI after logout
              window.location.reload();
            }}
            className="logout-btn"
            style={{
              marginLeft: '15px',
              padding: '5px 10px',
              backgroundColor: '#da27e0',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Logout
          </button>
        </div>
      ) : (
        <div className="auth-links">
          <Link
            to="/auth/login"
            className="button button--primary"
            style={{marginRight: '10px'}}
          >
            Login
          </Link>
          <Link
            to="/auth/signup"
            className="button button--secondary"
          >
            Sign Up
          </Link>
        </div>
      )}
    </div>
  );
};

export default AuthNavigation;