# User Schema and Test Cases for Better Auth Implementation

## User Schema

### Required Fields
- `id` (string): Unique identifier for the user
- `email` (string): User's email address (unique, required)
- `password` (string): User's hashed password (required)
- `name` (string): User's full name (required)

### Optional Fields
- `createdAt` (Date): Timestamp when the user was created
- `updatedAt` (Date): Timestamp when the user was last updated
- `emailVerified` (Date | null): Timestamp when email was verified (if required)
- `image` (string | null): URL to user's profile image

### Validation Rules
- Email must be a valid email format
- Password must be at least 8 characters long
- Name must be at least 2 characters long
- Email must be unique across all users

## Test Cases

### Unit Tests

#### Login Component Tests
1. **Render Test**
   - Given: Login component is mounted
   - When: Component renders
   - Then: Email and password fields are visible

2. **Form Validation Test - Empty Fields**
   - Given: User hasn't entered any data
   - When: User submits the form
   - Then: Appropriate validation errors are shown

3. **Form Validation Test - Invalid Email**
   - Given: User enters invalid email format
   - When: User submits the form
   - Then: Email validation error is shown

4. **Login Success Test**
   - Given: User enters valid credentials
   - When: User submits the form
   - Then: Login succeeds and user is redirected to dashboard

5. **Login Failure Test**
   - Given: User enters invalid credentials
   - When: User submits the form
   - Then: Appropriate error message is displayed

#### Signup Component Tests
1. **Render Test**
   - Given: Signup component is mounted
   - When: Component renders
   - Then: Name, email, password, and confirm password fields are visible

2. **Form Validation Test - Password Match**
   - Given: User enters mismatched passwords
   - When: User submits the form
   - Then: Password match validation error is shown

3. **Signup Success Test**
   - Given: User enters valid signup data
   - When: User submits the form
   - Then: Signup succeeds and user is redirected to dashboard

4. **Signup Failure Test**
   - Given: User enters invalid signup data
   - When: User submits the form
   - Then: Appropriate error message is displayed

### Integration Tests

1. **API Connectivity Test**
   - Given: Backend server is running
   - When: Frontend makes auth requests
   - Then: Requests are properly handled by Better Auth endpoints

2. **Session Creation Test**
   - Given: User successfully logs in
   - When: Session is created
   - Then: Valid session token is returned and stored

3. **Session Validation Test**
   - Given: User has active session
   - When: Accessing protected route
   - Then: User is allowed access

4. **Session Expiration Test**
   - Given: User's session has expired
   - When: Accessing protected route
   - Then: User is redirected to login page

### End-to-End Tests

1. **Complete Signup Flow**
   - Given: User navigates to signup page
   - When: User fills signup form and submits
   - Then: Account is created and user is logged in

2. **Complete Login Flow**
   - Given: User navigates to login page
   - When: User fills login form and submits
   - Then: User is logged in and redirected to dashboard

3. **Logout Flow**
   - Given: User is logged in
   - When: User clicks logout button
   - Then: Session is cleared and user is redirected to login page

4. **Protected Route Access**
   - Given: User is not logged in
   - When: User tries to access protected route
   - Then: User is redirected to login page

## Edge Cases

1. **Network Failure**
   - When: Network request fails during auth
   - Then: User sees appropriate error message

2. **Concurrent Requests**
   - When: Multiple auth requests happen simultaneously
   - Then: Only one request is processed at a time

3. **Invalid Session Token**
   - When: Session token is malformed or invalid
   - Then: User is logged out and redirected to login

4. **Rate Limiting**
   - When: Too many auth attempts from same IP
   - Then: Requests are temporarily blocked

## Performance Tests

1. **Login Performance**
   - Measure time to complete login process
   - Expected: < 2 seconds under normal conditions

2. **Signup Performance**
   - Measure time to complete signup process
   - Expected: < 3 seconds under normal conditions

3. **Session Validation Performance**
   - Measure time to validate session on protected routes
   - Expected: < 100ms under normal conditions