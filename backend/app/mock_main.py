"""
Mock FastAPI server for local development testing.
This provides mock responses without requiring external APIs.
"""

import json
from datetime import datetime
from typing import Dict, List
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API (Mock)",
    description="Mock API for local development testing",
    version="1.0.0",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Mock data structures
class QueryRequest(BaseModel):
    query: str
    session_id: str = None
    top_k: int = 5
    filters: Dict = None


class QueryResponse(BaseModel):
    answer: str
    session_id: str
    sources: List[Dict] = []
    confidence: float
    tokens_used: int = 0
    filter_message: str = None
    suggested_terms: List[str] = []


# Mock responses
MOCK_RESPONSES = {
    "hello": "Hello! I'm your Robotics AI Assistant. How can I help you today?",
    "ros": "ROS (Robot Operating System) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.",
    "isaac": "Isaac Sim is NVIDIA's robotics simulation platform built on Omniverse. It provides realistic physics simulation and rendering capabilities for testing and training robots in virtual environments.",
    "motor": "A motor is an actuator that converts electrical energy into mechanical motion. In robotics, motors are essential components that enable robots to move and interact with their environment.",
    "sensor": "Sensors are devices that detect and measure physical properties from the environment. Common robot sensors include cameras, LiDAR, IMUs, and touch sensors, which provide the robot with information about its surroundings.",
    "default": "That's an interesting question about robotics! Based on the textbook content, I would suggest checking the relevant modules for detailed information. The robotics textbook covers topics ranging from basic robot kinematics to advanced perception and control systems.",
}

# Mock sources
MOCK_SOURCES = [
    {
        "module": 1,
        "title": "Introduction to Robotics",
        "content": "Robotics is an interdisciplinary field involving computer science, engineering, and physics...",
        "url": "/docs/module1/introduction",
    },
    {
        "module": 2,
        "title": "Robot Kinematics",
        "content": "Kinematics is the study of motion without considering forces...",
        "url": "/docs/module2/kinematics",
    },
]


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API (Mock) is running",
        "status": "development mode",
        "docs": "/api/docs",
    }


@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "mode": "mock",
    }


@app.post("/api/v1/query")
async def query_chatbot(request: QueryRequest):
    """Main query endpoint with mock responses."""

    # Simple keyword matching for mock responses
    query_lower = request.query.lower()
    answer = MOCK_RESPONSES.get("default")

    for keyword, response in MOCK_RESPONSES.items():
        if keyword != "default" and keyword in query_lower:
            answer = response
            break

    # Generate a mock session ID if not provided
    session_id = request.session_id or f"mock_session_{datetime.now().timestamp()}"

    # Determine confidence based on match
    confidence = (
        0.8 if any(k in query_lower for k in MOCK_RESPONSES.keys() if k != "default") else 0.5
    )

    return QueryResponse(
        answer=answer,
        session_id=session_id,
        sources=MOCK_SOURCES,
        confidence=confidence,
        tokens_used=100,
        suggested_terms=["ROS 2", "Motion Planning", "SLAM", "Computer Vision"],
    )


@app.delete("/api/v1/chat/sessions/{session_id}")
async def clear_session(session_id: str):
    """Clear session history (mock implementation)."""
    return {"message": "Session cleared successfully", "session_id": session_id}


@app.get("/api/v1/modules")
async def get_modules():
    """Get available modules (mock data)."""
    return [
        {"id": 1, "title": "Introduction to Robotics", "difficulty": "beginner"},
        {"id": 2, "title": "Robot Kinematics", "difficulty": "intermediate"},
        {"id": 3, "title": "ROS 2 Fundamentals", "difficulty": "beginner"},
        {"id": 4, "title": "Computer Vision", "difficulty": "advanced"},
    ]


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
