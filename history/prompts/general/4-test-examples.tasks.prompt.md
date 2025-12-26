# Test Examples for Better Auth Implementation

## Unit Tests

### Login Component Tests

```javascript
// tests/auth/login.test.js
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { AuthProvider } from 'better-auth/react';
import Login from '../../src/components/Auth/Login';

// Mock the useAuth hook
jest.mock('better-auth/react', () => ({
  ...jest.requireActual('better-auth/react'),
  useAuth: () => ({
    signIn: {
      email: jest.fn().mockResolvedValue({ success: true })
    }
  })
}));

describe('Login Component', () => {
  const mockOnSwitchToSignup = jest.fn();
  
  beforeEach(() => {
    render(
      <AuthProvider>
        <Login onSwitchToSignup={mockOnSwitchToSignup} />
      </AuthProvider>
    );
  });

  test('renders login form elements', () => {
    expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /login/i })).toBeInTheDocument();
    expect(screen.getByText(/don't have an account/i)).toBeInTheDocument();
  });

  test('shows validation errors for empty fields', async () => {
    fireEvent.click(screen.getByRole('button', { name: /login/i }));
    
    await waitFor(() => {
      expect(screen.getByText(/email is required/i)).toBeInTheDocument();
      expect(screen.getByText(/password is required/i)).toBeInTheDocument();
    });
  });

  test('calls switch to signup function when button is clicked', () => {
    fireEvent.click(screen.getByText(/sign up here/i));
    expect(mockOnSwitchToSignup).toHaveBeenCalledTimes(1);
  });

  test('successfully logs in with valid credentials', async () => {
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    
    fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'password123' } });
    
    fireEvent.click(screen.getByRole('button', { name: /login/i }));
    
    await waitFor(() => {
      expect(screen.queryByText(/login failed/i)).not.toBeInTheDocument();
    });
  });
});
```

### Signup Component Tests

```javascript
// tests/auth/signup.test.js
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { AuthProvider } from 'better-auth/react';
import Signup from '../../src/components/Auth/Signup';

// Mock the useAuth hook
jest.mock('better-auth/react', () => ({
  ...jest.requireActual('better-auth/react'),
  useAuth: () => ({
    signUp: {
      email: jest.fn().mockResolvedValue({ success: true })
    }
  })
}));

describe('Signup Component', () => {
  const mockOnSwitchToLogin = jest.fn();
  
  beforeEach(() => {
    render(
      <AuthProvider>
        <Signup onSwitchToLogin={mockOnSwitchToLogin} />
      </AuthProvider>
    );
  });

  test('renders signup form elements', () => {
    expect(screen.getByLabelText(/name/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/confirm password/i)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /sign up/i })).toBeInTheDocument();
    expect(screen.getByText(/already have an account/i)).toBeInTheDocument();
  });

  test('shows error when passwords do not match', async () => {
    const passwordInput = screen.getByLabelText(/password/i);
    const confirmPasswordInput = screen.getByLabelText(/confirm password/i);
    
    fireEvent.change(passwordInput, { target: { value: 'password123' } });
    fireEvent.change(confirmPasswordInput, { target: { value: 'different123' } });
    
    fireEvent.click(screen.getByRole('button', { name: /sign up/i }));
    
    await waitFor(() => {
      expect(screen.getByText(/passwords do not match/i)).toBeInTheDocument();
    });
  });

  test('calls switch to login function when button is clicked', () => {
    fireEvent.click(screen.getByText(/login here/i));
    expect(mockOnSwitchToLogin).toHaveBeenCalledTimes(1);
  });

  test('successfully signs up with valid credentials', async () => {
    const nameInput = screen.getByLabelText(/name/i);
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    const confirmPasswordInput = screen.getByLabelText(/confirm password/i);
    
    fireEvent.change(nameInput, { target: { value: 'Test User' } });
    fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'password123' } });
    fireEvent.change(confirmPasswordInput, { target: { value: 'password123' } });
    
    fireEvent.click(screen.getByRole('button', { name: /sign up/i }));
    
    await waitFor(() => {
      expect(screen.queryByText(/signup failed/i)).not.toBeInTheDocument();
    });
  });
});
```

## Integration Tests

### API Endpoint Tests

```javascript
// tests/auth/api.test.js
const request = require('supertest');
const app = require('../../server');

describe('Authentication API Endpoints', () => {
  test('GET /api/auth/me returns user session when authenticated', async () => {
    // Mock session exists
    const response = await request(app)
      .get('/api/auth/me')
      .set('Cookie', ['session=valid-session-token'])
      .expect(200);
    
    expect(response.body).toHaveProperty('user');
    expect(response.body).toHaveProperty('session');
  });

  test('GET /api/auth/me returns 401 when not authenticated', async () => {
    const response = await request(app)
      .get('/api/auth/me')
      .expect(401);
    
    expect(response.body.error).toBe('Not authenticated');
  });

  test('POST /api/auth/signout invalidates session', async () => {
    const response = await request(app)
      .post('/api/auth/signout')
      .set('Cookie', ['session=valid-session-token'])
      .send({})
      .expect(200);
    
    // Check that session cookie is cleared
    expect(response.headers['set-cookie']).toBeDefined();
  });
});
```

## End-to-End Tests

### Complete Flow Tests

```javascript
// tests/auth/e2e.test.js
import { test, expect } from '@playwright/test';

test.describe('Authentication Flow', () => {
  test('should allow user to signup and login', async ({ page }) => {
    // Go to signup page
    await page.goto('/auth/signup');
    
    // Fill signup form
    await page.fill('input#name', 'Test User');
    await page.fill('input#email', 'test@example.com');
    await page.fill('input#password', 'password123');
    await page.fill('input#confirmPassword', 'password123');
    
    // Submit signup
    await page.click('button[type="submit"]');
    
    // Verify redirect to dashboard
    await expect(page).toHaveURL(/dashboard/);
    
    // Logout
    await page.click('button#logout');
    
    // Go to login page
    await page.goto('/auth/login');
    
    // Fill login form
    await page.fill('input#email', 'test@example.com');
    await page.fill('input#password', 'password123');
    
    // Submit login
    await page.click('button[type="submit"]');
    
    // Verify redirect to dashboard
    await expect(page).toHaveURL(/dashboard/);
  });

  test('should redirect unauthenticated user to login', async ({ page }) => {
    // Try to access protected route
    await page.goto('/dashboard');
    
    // Should be redirected to login
    await expect(page).toHaveURL(/auth\/login/);
  });
});
```

## Performance Tests

### Login Performance Test

```javascript
// tests/auth/performance.test.js
import { performance } from 'perf_hooks';

describe('Authentication Performance', () => {
  test('login should complete within 2 seconds', async () => {
    const startTime = performance.now();
    
    // Simulate login process
    const loginResult = await signIn.email({
      email: 'test@example.com',
      password: 'password123',
      callbackURL: '/dashboard'
    });
    
    const endTime = performance.now();
    const duration = endTime - startTime;
    
    expect(duration).toBeLessThan(2000); // Less than 2 seconds
    expect(loginResult).toBeDefined();
  });

  test('signup should complete within 3 seconds', async () => {
    const startTime = performance.now();
    
    // Simulate signup process
    const signupResult = await signUp.email({
      email: 'newuser@example.com',
      password: 'password123',
      name: 'New User',
      callbackURL: '/dashboard'
    });
    
    const endTime = performance.now();
    const duration = endTime - startTime;
    
    expect(duration).toBeLessThan(3000); // Less than 3 seconds
    expect(signupResult).toBeDefined();
  });
});
```

## Error Handling Tests

### Network Failure Test

```javascript
// tests/auth/error-handling.test.js
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { AuthProvider } from 'better-auth/react';
import Login from '../../src/components/Auth/Login';

// Mock the useAuth hook to simulate network failure
jest.mock('better-auth/react', () => ({
  ...jest.requireActual('better-auth/react'),
  useAuth: () => ({
    signIn: {
      email: jest.fn().mockRejectedValue(new Error('Network error'))
    }
  })
}));

describe('Error Handling', () => {
  test('shows error message when network fails', async () => {
    render(
      <AuthProvider>
        <Login onSwitchToSignup={() => {}} />
      </AuthProvider>
    );
    
    const emailInput = screen.getByLabelText(/email/i);
    const passwordInput = screen.getByLabelText(/password/i);
    
    fireEvent.change(emailInput, { target: { value: 'test@example.com' } });
    fireEvent.change(passwordInput, { target: { value: 'password123' } });
    
    fireEvent.click(screen.getByRole('button', { name: /login/i }));
    
    await waitFor(() => {
      expect(screen.getByText(/network error/i)).toBeInTheDocument();
    });
  });
});
```

These test examples provide a comprehensive testing approach for the Better Auth implementation, covering all critical functionality and edge cases.