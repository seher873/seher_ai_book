// src/components/Auth/AuthProvider.jsx
// This file now serves as a wrapper for the main auth context
import React from 'react';
import { AuthProvider as CustomAuthProvider } from '../../auth-client';

// Wrapper component that uses our custom AuthProvider
const AuthProviderWrapper = ({ children }) => {
  return (
    <CustomAuthProvider>
      {children}
    </CustomAuthProvider>
  );
};

export default AuthProviderWrapper;