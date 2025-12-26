// Authentication tests for Better Auth implementation
const request = require('supertest');
const express = require('express');
const { toNodeHandler, fromNodeHeaders } = require('better-auth/node');
const { auth } = require('../src/auth.js');

// Create an express app to test the auth endpoints
const app = express();
app.use(express.json());

// Mount Better Auth routes
app.all('/api/auth/*', async (req, res) => {
  // Handle body parsing for Better Auth
  if (req.method === 'POST' && req.headers['content-type']?.includes('application/json')) {
    let body = '';
    req.on('data', chunk => {
      body += chunk.toString();
    });

    req.on('end', async () => {
      try {
        req.body = JSON.parse(body);
        await toNodeHandler(auth)(req, res);
      } catch (e) {
        res.status(400).json({ error: 'Invalid JSON' });
      }
    });
  } else {
    await toNodeHandler(auth)(req, res);
  }
});

// Test suite for authentication
describe('Better Auth Implementation', () => {
  describe('Signup Flow', () => {
    test('should allow user to sign up with background information', async () => {
      const response = await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'test@example.com',
          password: 'securePassword123',
          name: 'Test User',
          // Background information as per requirements
          softwareBackground: {
            level: 'intermediate',
            languages: ['JavaScript', 'Python'],
            interests: ['AI', 'Robotics']
          },
          hardwareBackground: {
            experience: 'beginner',
            tools: ['Arduino', 'Raspberry Pi']
          }
        })
        .expect(200);
      
      expect(response.body).toHaveProperty('session');
      expect(response.body).toHaveProperty('user');
      expect(response.body.user.email).toBe('test@example.com');
    });

    test('should validate required background information during signup', async () => {
      const response = await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'test2@example.com',
          password: 'securePassword123',
          name: 'Test User 2'
          // Missing background information
        })
        .expect(400);
      
      expect(response.body).toHaveProperty('error');
    });

    test('should store background information in user profile', async () => {
      // First sign up a user
      const signupResponse = await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'test3@example.com',
          password: 'securePassword123',
          name: 'Test User 3',
          softwareBackground: {
            level: 'advanced',
            languages: ['C++', 'ROS'],
            interests: ['Computer Vision', 'Navigation']
          },
          hardwareBackground: {
            experience: 'intermediate',
            tools: ['Oscilloscope', 'Multimeter', 'Soldering Iron']
          }
        })
        .expect(200);
      
      // Then verify the user data includes background information
      const userId = signupResponse.body.user.id;
      expect(signupResponse.body.user).toHaveProperty('softwareBackground');
      expect(signupResponse.body.user).toHaveProperty('hardwareBackground');
      expect(signupResponse.body.user.softwareBackground.level).toBe('advanced');
      expect(signupResponse.body.user.hardwareBackground.experience).toBe('intermediate');
    });
  });

  describe('Signin Flow', () => {
    test('should allow existing user to sign in', async () => {
      // First sign up a user
      await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'signin-test@example.com',
          password: 'securePassword123',
          name: 'Signin Test',
          softwareBackground: {
            level: 'beginner',
            languages: ['Python'],
            interests: ['Machine Learning']
          },
          hardwareBackground: {
            experience: 'none',
            tools: []
          }
        })
        .expect(200);

      // Then sign in
      const response = await request(app)
        .post('/api/auth/signin/email')
        .send({
          email: 'signin-test@example.com',
          password: 'securePassword123'
        })
        .expect(200);
      
      expect(response.body).toHaveProperty('session');
      expect(response.body).toHaveProperty('user');
    });

    test('should return error for invalid credentials', async () => {
      const response = await request(app)
        .post('/api/auth/signin/email')
        .send({
          email: 'nonexistent@example.com',
          password: 'wrongPassword'
        })
        .expect(400);
      
      expect(response.body).toHaveProperty('error');
    });
  });

  describe('Session Management', () => {
    test('should get current user session', async () => {
      // First sign up a user
      const signupResponse = await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'session-test@example.com',
          password: 'securePassword123',
          name: 'Session Test',
          softwareBackground: {
            level: 'expert',
            languages: ['JavaScript', 'C++', 'Python'],
            interests: ['AI', 'Robotics', 'Control Systems']
          },
          hardwareBackground: {
            experience: 'advanced',
            tools: ['Oscilloscope', 'Logic Analyzer', '3D Printer']
          }
        })
        .expect(200);

      // Get session with the token
      const token = signupResponse.body.session.token;
      const sessionResponse = await request(app)
        .get('/api/auth/session')
        .set('Authorization', `Bearer ${token}`)
        .expect(200);
      
      expect(sessionResponse.body).toHaveProperty('user');
      expect(sessionResponse.body.user.softwareBackground.level).toBe('expert');
      expect(sessionResponse.body.user.hardwareBackground.experience).toBe('advanced');
    });

    test('should sign out user', async () => {
      // First sign up a user
      const signupResponse = await request(app)
        .post('/api/auth/signup/email')
        .send({
          email: 'signout-test@example.com',
          password: 'securePassword123',
          name: 'Signout Test',
          softwareBackground: {
            level: 'intermediate',
            languages: ['Python', 'C'],
            interests: ['Embedded Systems']
          },
          hardwareBackground: {
            experience: 'intermediate',
            tools: ['Soldering Iron', 'Multimeter']
          }
        })
        .expect(200);

      // Sign out
      const token = signupResponse.body.session.token;
      const signoutResponse = await request(app)
        .post('/api/auth/signout')
        .set('Authorization', `Bearer ${token}`)
        .send({})
        .expect(200);
      
      expect(signoutResponse.body).toHaveProperty('success');
    });
  });
});