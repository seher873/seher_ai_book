// src/components/Auth/Signin.jsx
import React, { useState } from 'react';
import { useAuth } from './AuthProvider'; // Using consistent auth system

const Signin = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const { signin, error } = useAuth(); // Using the custom auth context
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      setLoading(true);
      await signin(email, password); // Using the custom signin function
      alert('Signin successful!');
      // Optionally redirect to dashboard
      window.location.href = '/dashboard';
    } catch (err) {
      const errorMsg = err?.message || String(err) || 'An error occurred during signin';
      console.error('Signin error:', errorMsg);
      alert(errorMsg);
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
      </form>
    </div>
  );
};

export default Signin;