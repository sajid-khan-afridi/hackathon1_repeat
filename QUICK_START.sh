#!/bin/bash

# RAG Chatbot Quick Start Script
# This script helps you get the RAG chatbot system running quickly

echo "🤖 RAG Chatbot Quick Start"
echo "=========================="

# Check if .env file exists
if [ ! -f .env ]; then
    echo "⚠️  .env file not found. Creating from template..."
    if [ -f .env.example ]; then
        cp .env.example .env
        echo "✅ Created .env file from .env.example"
        echo "❗ Please edit .env with your actual API keys and configuration"
        echo ""
    else
        echo "❌ .env.example file not found!"
        exit 1
    fi
fi

# Function to check if a service is running
check_service() {
    local service=$1
    local port=$2
    if nc -z localhost $port 2>/dev/null; then
        echo "✅ $service is running on port $port"
        return 0
    else
        echo "❌ $service is not running on port $port"
        return 1
    fi
}

# Check prerequisites
echo "🔍 Checking prerequisites..."

# Check PostgreSQL
check_service "PostgreSQL" 5432
POSTGRES_RUNNING=$?

# Check Qdrant
check_service "Qdrant" 6333
QDRANT_RUNNING=$?

# Install backend dependencies
echo ""
echo "📦 Installing Python dependencies..."
cd backend
if [ -f requirements.txt ]; then
    pip install -r requirements.txt
    echo "✅ Python dependencies installed"
else
    echo "❌ requirements.txt not found in backend directory!"
fi

# Initialize database
echo ""
echo "🗄️  Initializing database..."
if [ -f startup.py ]; then
    python startup.py
    echo "✅ Database initialized"
else
    echo "❌ startup.py not found!"
fi

# Start backend server
echo ""
echo "🚀 Starting backend server..."
echo "Choose your startup mode:"
echo "1) Development mode (with auto-reload)"
echo "2) Production mode"
echo "3) Docker Compose"
read -p "Enter your choice (1-3): " choice

case $choice in
    1)
        echo "Starting in development mode..."
        uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
        ;;
    2)
        echo "Starting in production mode..."
        uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
        ;;
    3)
        echo "Starting with Docker Compose..."
        cd ..
        docker-compose up -d
        ;;
    *)
        echo "Invalid choice. Starting in development mode..."
        uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
        ;;
esac

# Display next steps
echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Open http://localhost:8000/docs to view API documentation"
echo "2. Check health at http://localhost:8000/health"
echo "3. Integrate the ChatbotWidget into your frontend"
echo "4. Populate Qdrant with your textbook content"
echo ""
echo "Example integration code:"
echo "import ChatbotWidget from './src/components/ChatbotWidget';"
echo "<ChatbotWidget apiUrl=\"http://localhost:8000\" />"