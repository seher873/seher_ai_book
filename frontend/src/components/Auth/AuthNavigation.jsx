import React from 'react';
import { useAuth } from '../../auth-client';
import Link from '@docusaurus/Link';

const AuthNavigation = () => {
  const { session, isPending, signOut } = useAuth();

  if (isPending) {
    return <div>Loading...</div>;
  }

  return (
    <div className="auth-nav">
      {session ? (
        <div className="user-menu">
          <span>Welcome, {session.user.name}!</span>
          <button
            onClick={() => {
              signOut();
              // Redirect to home after logout
              window.location.href = '/';
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