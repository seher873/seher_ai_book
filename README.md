# Physical AI & Humanoid Robotics Textbook

A comprehensive interactive textbook platform with RAG chatbot and authentication system for learning Physical AI and Humanoid Robotics.

## Project Overview

This project combines a Docusaurus-based educational platform with an AI-powered RAG (Retrieval-Augmented Generation) chatbot to provide an enhanced learning experience for Physical AI and Humanoid Robotics.

## Features

- **Interactive Textbook**: Built with Docusaurus for rich content presentation
- **RAG Chatbot**: AI-powered assistant to answer questions from textbook content
- **User Authentication**: Secure signup/signin with user profiles
- **Labs & Assessments**: Hands-on exercises and knowledge checks
- **Glossary & Appendices**: Comprehensive reference materials
- **Responsive Design**: Works on desktop and mobile devices

## Project Structure

```
my_ai_book/
├── frontend/                # Frontend application
│   ├── src/                # React components
│   │   ├── components/    # Reusable components
│   │   │   ├── Auth/     # Authentication components
│   │   │   ├── Chatbot/  # RAG chatbot components
│   │   │   ├── Assessment/ # Assessment components
│   │   │   ├── Lab/      # Lab components
│   │   │   ├── Glossary/ # Glossary components
│   │   │   └── Navigation/ # Navigation components
│   │   ├── pages/        # Custom pages
│   │   ├── css/          # Stylesheets
│   │   ├── theme/        # Theme customization
│   │   └── utils/        # Frontend utilities
│   │
│   ├── docs/             # Book content (Markdown)
│   │   ├── ch1-introduction/  # Chapter 1: ROS2 Fundamentals
│   │   ├── ch2-perception/    # Chapter 2: Sensors & Perception
│   │   ├── ch3-simulation/    # Chapter 3: Simulation
│   │   ├── ch4-isaac/         # Chapter 4: NVIDIA Isaac
│   │   ├── ch5-vla/           # Chapter 5: VLA Models
│   │   ├── ch6-control/       # Chapter 6: Control Systems
│   │   ├── ch7-navigation/    # Chapter 7: Navigation
│   │   ├── appendices/        # Additional resources
│   │   └── labs/              # Lab exercises
│   │
│   ├── static/           # Static assets (images, files)
│   ├── tests/            # Frontend tests
│   ├── docusaurus.config.js  # Docusaurus configuration
│   ├── sidebars.js          # Sidebar configuration
│   ├── package.json         # Node dependencies
│   ├── babel.config.js      # Babel configuration
│   ├── jest.config.js       # Jest testing
│   └── server.js            # Production server
│
├── backend/              # FastAPI backend (Python)
│   ├── auth/            # Authentication module
│   │   ├── config.py   # Auth configuration
│   │   ├── database.py # Database connection
│   │   ├── dependencies.py # FastAPI dependencies
│   │   ├── main.py     # Auth routes
│   │   ├── models.py   # SQLAlchemy models
│   │   ├── schemas.py  # Pydantic schemas
│   │   ├── services.py # Business logic
│   │   └── utils.py    # JWT & password utilities
│   │
│   ├── scripts/         # Utility scripts
│   │   ├── check_qdrant_data.py
│   │   ├── ingest_all_content.py
│   │   └── init_db.py
│   │
│   ├── routes/          # API routes (future use)
│   ├── services/        # Business logic (future use)
│   ├── utils/           # Shared utilities (future use)
│   ├── main.py          # Main FastAPI app
│   ├── requirements.txt # Python dependencies
│   ├── Dockerfile       # Docker configuration
│   └── README.md        # Backend documentation
│
├── .github/             # GitHub configuration
├── .gitignore           # Git ignore rules
└── README.md            # This file
```

## Tech Stack

### Backend
- **FastAPI**: Modern Python web framework
- **Qdrant**: Vector database for semantic search
- **Cohere**: Embedding generation for RAG
- **OpenRouter**: LLM integration (Gemini 1.5 Flash)
- **SQLAlchemy**: Database ORM
- **SQLite**: Development database
- **JWT**: Token-based authentication
- **bcrypt**: Password hashing

### Frontend
- **React 18**: UI framework
- **Docusaurus 3**: Documentation framework
- **better-auth**: Authentication library
- **Express**: Node.js server
- **Jest**: Testing framework
- **Babel**: JavaScript transpiler

## Getting Started

### Prerequisites
- Node.js 18+
- Python 3.9+
- npm or yarn

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/seher873/seher_ai_book.git
cd my_ai_book
```

2. **Install frontend dependencies**
```bash
cd frontend
npm install
```

3. **Setup backend**
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

4. **Configure environment variables**

Backend (.env in backend/):
```env
COHERE_API_KEY=your_cohere_api_key
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_MODEL=xiaomi/mimo-v2-flash
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
JWT_SECRET=your_jwt_secret
DATABASE_URL=sqlite:///./app.db
```

5. **Initialize database**
```bash
cd backend
python scripts/init_db.py
```

### Running the Application

#### Development Mode

**Frontend** (Port 3000):
```bash
cd frontend
npm start
```

**Backend** (Port 8000):
```bash
cd backend
python main.py
```

#### Production Mode

```bash
cd frontend
npm run serve:prod
```

The production server runs on port 3000 and proxies backend requests to port 8000.

## API Endpoints

### Health & Info
- `GET /` - Root endpoint
- `GET /health` - Health check

### RAG Chatbot
- `POST /chat` - Query the textbook chatbot
- `POST /ingest` - Ingest documents into vector database

### Authentication
- `POST /auth/signup` - User registration
- `POST /auth/signin` - User login

Full API documentation: http://localhost:8000/docs

## Development

### Frontend Scripts

```bash
cd frontend
npm start              # Start development server
npm run build          # Build for production
npm run serve          # Serve production build
npm run serve:prod     # Serve with backend proxy
npm test               # Run tests
npm run test:auth      # Run auth tests
```

### Backend Scripts

```bash
cd backend
python main.py                        # Run backend server
python scripts/init_db.py             # Initialize database
python scripts/ingest_all_content.py  # Ingest content to Qdrant
python scripts/check_qdrant_data.py   # Check Qdrant data
```

### Code Organization

- **Frontend**: All UI, components, docs, and static files in `frontend/`
- **Backend**: All Python code, APIs, and scripts in `backend/`
- **Clear separation**: Frontend and backend are completely independent
- **Modular structure**: Components organized by feature

## Testing

```bash
# Frontend tests
cd frontend
npm test
npm run test:auth

# Backend tests
cd backend
pytest
```

## Deployment

### Frontend (Netlify)
- Connected to GitHub repository
- Auto-deploys on push to main branch
- URL: https://seher-robotic-book.netlify.app
- Build directory: `frontend/build`

### Backend (Hugging Face Spaces)
- Dockerized deployment
- Set environment variables in Secrets
- See backend/README.md for details

## Architecture

### RAG Pipeline
```
User Query → Cohere Embedding → Qdrant Search → Context Retrieval →
OpenRouter LLM → Response
```

### Authentication Flow
```
User Input → Password Hash → JWT Generation → Token Storage →
Protected Routes Access
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Write/update tests
5. Update documentation
6. Submit a pull request

## Project Goals

- Provide comprehensive Physical AI education
- Interactive learning with AI assistance
- Hands-on labs and assessments
- Clean, maintainable codebase
- Scalable architecture
- Clear frontend/backend separation

## License

[Your License Here]

## Contact

- GitHub: [@seher873](https://github.com/seher873)
- Repository: [seher_ai_book](https://github.com/seher873/seher_ai_book)

## Acknowledgments

Built with:
- Docusaurus by Facebook
- FastAPI by Sebastián Ramírez
- Cohere for embeddings
- Qdrant for vector search
- OpenRouter for LLM access
