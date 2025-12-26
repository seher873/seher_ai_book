// src/components/Auth/Signin.jsx
import React, { useState } from 'react';
import { useAuth } from './AuthProvider';

const Signin = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const { signin, error } = useAuth();
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      setLoading(true);
      await signin(email, password);
      alert('Signin successful!');
    } catch (err) {
      console.error('Signin error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <h2>Sign In</h2>
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
        <button type="submit" disabled={loading}>
          {loading ? 'Signing in...' : 'Sign In'}
        </button>
        {error && <div className="error-message">{error}</div>}
      </form>
    </div>
  );
};

export default Signin;