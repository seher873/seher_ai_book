# Quickstart Guide: Module 2 Content Generation

## Prerequisites

- Node.js v20+ installed
- Git installed and configured
- Access to the project repository
- Appropriate user role (author, editor, or admin)

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/my-ai-book.git
   cd my-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run dev
   ```

4. The application will be available at `http://localhost:3000`

## Creating Module 2 Content

1. Ensure you have author privileges in the system.

2. Access the content creation interface:
   - Navigate to `/create-content` in the application
   - Select "Module 2: Digital Twin - Gazebo & Unity" as the target module
   - Choose the content type (lesson, chapter, exercise, or example)

3. Fill in the content details:
   - Title
   - Markdown content
   - Associated chapter (1-5)
   - Learning objectives
   - Prerequisites
   - Hands-on examples (if applicable)

4. Save the content as "draft" initially.

5. Submit for review when ready:
   - Update the content status to "under_review"
   - Notify the assigned editor

## Content Structure

Module 2 should follow this 5-chapter structure:

1. **Chapter 1**: Introduction to Digital Twins
2. **Chapter 2**: Gazebo Fundamentals
3. **Chapter 3**: Unity Robotics Simulation
4. **Chapter 4**: ROS 2 Integration with Simulation
5. **Chapter 5**: Hands-on Labs & Troubleshooting

Each chapter should include:
- Clear learning objectives
- Interactive text with hands-on examples
- Relevant assessments (quizzes or projects)
- Appropriate prerequisites

## Reviewing Content

1. If you have editor privileges, review content marked as "under_review"
2. Check for:
   - Content accuracy (aligns with Digital Twin, Gazebo, Unity topics)
   - Educational clarity (appropriate for beginner-intermediate audience)
   - Ethical considerations (addressed where relevant)
   - Practical examples (included and functional)
   - Proper structure and organization

3. Update the content status to either:
   - "approved" - if all good
   - "draft" - if significant changes needed

## Running the Backend API

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install backend dependencies:
   ```bash
   npm install
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

4. Start the backend server:
   ```bash
   npm run start
   ```

## API Usage Examples

### Creating new content
```bash
curl -X POST https://api.my-ai-book.com/v1/modules/2/contents \
  -H "Authorization: Bearer <your-jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Introduction to Gazebo Simulation",
    "content": "# Gazebo Basics\n\nGazebo is a robotics simulator...",
    "contentType": "lesson",
    "learningObjectives": ["Understand Gazebo fundamentals", "Create basic simulation environments"],
    "chapterNumber": 2
  }'
```

### Updating content status
```bash
curl -X PUT https://api.my-ai-book.com/v1/modules/2/contents/{contentId}/status \
  -H "Authorization: Bearer <your-jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "status": "under_review",
    "comment": "Content ready for editorial review"
  }'
```

## Testing

1. Run backend tests:
   ```bash
   npm run test
   ```

2. Run frontend tests:
   ```bash
   cd frontend && npm run test
   ```

## Deploying

1. Build the application:
   ```bash
   npm run build
   ```

2. Deploy to your preferred platform (details depend on your infrastructure)

## Troubleshooting

- If content fails to save, ensure you have the correct user role
- If API calls return 401 errors, check your JWT token validity
- For performance issues, verify that content loads within 2-3 seconds as required
- For authentication/authorization problems, contact an admin to verify your role