# Physical AI & Humanoid Robotics Textbook

This repository contains the content for the "Physical AI & Humanoid Robotics" textbook, a comprehensive educational resource for university-level courses covering robotics, simulation, and artificial intelligence integration.

## Overview

This textbook combines theoretical foundations with practical implementation in modern robotic frameworks, emphasizing hands-on learning through structured lab exercises. It's designed to provide undergraduate and graduate students with the knowledge and skills necessary to develop physical AI systems using state-of-the-art tools and platforms.

## Technology Stack

- **Documentation Platform**: Docusaurus v3.x
- **Robotics Framework**: ROS2 Humble Hawksbill
- **Simulation**: Gazebo Garden and Unity ML-Agents
- **Platform**: NVIDIA Isaac for advanced perception and navigation
- **Language**: Markdown for content, with code examples in Python/C++

## Getting Started

To run the textbook locally:

1. Clone the repository:
   ```bash
   git clone https://github.com/seher873/seher_ai_book.git
   ```

2. Navigate to the project directory:
   ```bash
   cd physical-ai-textbook
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

4. Run the development server:
   ```bash
   npm start
   ```

5. Open [http://localhost:3000](http://localhost:3000) to view the textbook in your browser.

## Chatbot Integration

The textbook includes an AI chatbot that can answer questions about the textbook content using a Retrieval-Augmented Generation (RAG) system. To use the full chatbot functionality:

1. Set up the backend:
   - Navigate to the backend directory: `cd backend`
   - Install Python dependencies: `pip install -r requirements.txt`
   - Set up environment variables in a `.env` file with your API keys
   - Ingest the textbook content: `python ingest_docs.py`
   - Start the backend: `python -m uvicorn main:app --reload`

2. The frontend will automatically connect to the backend through API proxying.

## Features

The textbook includes the following features to enhance learning:

- **Structured Content**: Organized by chapters and modules covering both theoretical and practical aspects
- **Interactive Assessments**: Knowledge checks at the end of chapters
- **Hands-on Labs**: Practical exercises with real-world robotics applications
- **Glossary**: Comprehensive definitions of AI and robotics terminology
- **Appendices**: Supplementary material for deeper exploration of advanced topics
- **Hardware Guidelines**: Recommendations for computing requirements and setup
- **AI Chatbot**: Interactive assistant that can answer questions about textbook content using RAG (Retrieval-Augmented Generation)

## Structure

The textbook content is organized in the `docs/` directory with the following structure:

- `ch1-introduction/` - Introduction to Physical AI and ROS2 Framework
- `ch2-perception/` - Robot Perception Systems
- `ch3-simulation/` - Simulation Environments - Gazebo and Unity
- `ch4-isaac/` - Isaac Robotics Platform
- `ch5-vla/` - Vision Language Action (VLA) Models
- `ch6-control/` - Robot Control and Manipulation
- `ch7-navigation/` - Navigation Systems
- `ch8-integration/` - Integration and System Design
- `ch9-ethics/` - Ethics and Social Implications
- `modules/` - Technical deep-dive modules
- `labs/` - Hands-on lab exercises
- `glossary/` - Comprehensive glossary of AI and robotics terms
- `appendices/` - Supplementary material

## Contributing

We welcome contributions to improve the textbook content. Please follow these steps:

1. Fork the repository
2. Create a new branch for your changes
3. Make your changes
4. Submit a pull request with a clear description of your changes

## License

This textbook is licensed under [specify license].

## Contact

For questions or feedback, please contact [contact information].