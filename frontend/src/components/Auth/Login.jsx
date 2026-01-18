import React, { useState } from 'react';
import { useAuth } from './AuthProvider'; // Using consistent auth system

const Login = ({ onSwitchToSignup }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { signin } = useAuth(); // Using the custom auth context

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      await signin(email, password); // Using the custom signin function
      // Redirect to dashboard on success
      if (typeof window !== 'undefined') {
        window.location.href = '/dashboard';
      }
    } catch (err) {
      const errorMsg = err?.message || String(err) || 'An error occurred during login';
      setError(errorMsg);
    }
  };

  return (
    <div className="auth-container">
      <h2>Login</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>
        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>
        {error && <div className="error-message">{error}</div>}
        <button type="submit" className="btn btn-primary">Login</button>
      </form>
      <div className="auth-switch">
        Don't have an account?{' '}
        <button
          type="button"
          onClick={onSwitchToSignup}
          className="switch-btn"
        >
          Sign up here
        </button>
      </div>
    </div>
  );
};

export default Login;