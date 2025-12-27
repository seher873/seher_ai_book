import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Login from '../Auth/Login';
import Signup from '../Auth/Signup';

const AuthPage = () => {
  const [isLoginView, setIsLoginView] = useState(true);

  const switchToLogin = () => setIsLoginView(true);
  const switchToSignup = () => setIsLoginView(false);

  return (
    <Layout title={isLoginView ? "Login" : "Sign Up"} description={isLoginView ? "Login to your account" : "Create a new account"}>
      <main className="auth-page">
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '70vh',
          padding: '20px'
        }}>
          <div style={{
            width: '100%',
            maxWidth: '400px',
            padding: '30px',
            border: '1px solid #ddd',
            borderRadius: '8px',
            boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
          }}>
            {isLoginView ? (
              <Login onSwitchToSignup={switchToSignup} />
            ) : (
              <Signup onSwitchToLogin={switchToLogin} />
            )}
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default AuthPage;