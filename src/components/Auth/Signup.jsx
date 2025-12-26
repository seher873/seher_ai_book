// src/components/Auth/Signup.jsx
import React, { useState } from 'react';
import { useAuth } from './AuthProvider';

const Signup = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('');
  const [softwareLanguages, setSoftwareLanguages] = useState('');
  const [hardwareExperience, setHardwareExperience] = useState('');
  const [hardwareTools, setHardwareTools] = useState('');
  const { signup, error } = useAuth();
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (password !== confirmPassword) {
      alert("Passwords don't match");
      return;
    }

    try {
      setLoading(true);
      
      // Parse the languages and tools as arrays
      const softwareLangArray = softwareLanguages.split(',').map(lang => lang.trim()).filter(lang => lang);
      const hardwareToolsArray = hardwareTools.split(',').map(tool => tool.trim()).filter(tool => tool);
      
      await signup(
        email,
        password,
        name,
        {
          level: softwareLevel,
          languages: softwareLangArray
        },
        {
          experience: hardwareExperience,
          tools: hardwareToolsArray
        }
      );
      
      alert('Signup successful!');
    } catch (err) {
      console.error('Signup error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <h2>Sign Up</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="name">Name:</label>
          <input
            type="text"
            id="name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
          />
        </div>
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
        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            type="password"
            id="confirmPassword"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            required
          />
        </div>
        
        {/* Software Background */}
        <div className="form-group">
          <h3>Software Background</h3>
          <label htmlFor="softwareLevel">Experience Level:</label>
          <input
            type="text"
            id="softwareLevel"
            value={softwareLevel}
            onChange={(e) => setSoftwareLevel(e.target.value)}
            placeholder="e.g. beginner, intermediate, advanced"
          />
          <label htmlFor="softwareLanguages">Programming Languages (comma separated):</label>
          <input
            type="text"
            id="softwareLanguages"
            value={softwareLanguages}
            onChange={(e) => setSoftwareLanguages(e.target.value)}
            placeholder="e.g. Python, JavaScript, C++"
          />
        </div>
        
        {/* Hardware Background */}
        <div className="form-group">
          <h3>Hardware Background</h3>
          <label htmlFor="hardwareExperience">Experience Level:</label>
          <input
            type="text"
            id="hardwareExperience"
            value={hardwareExperience}
            onChange={(e) => setHardwareExperience(e.target.value)}
            placeholder="e.g. beginner, intermediate, advanced"
          />
          <label htmlFor="hardwareTools">Tools & Hardware (comma separated):</label>
          <input
            type="text"
            id="hardwareTools"
            value={hardwareTools}
            onChange={(e) => setHardwareTools(e.target.value)}
            placeholder="e.g. Arduino, Raspberry Pi, ROS"
          />
        </div>
        
        <button type="submit" disabled={loading}>
          {loading ? 'Signing up...' : 'Sign Up'}
        </button>
        
        {error && <div className="error-message">{error}</div>}
      </form>
    </div>
  );
};

export default Signup;