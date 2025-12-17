from flask import Flask, request, jsonify
from flask_cors import CORS
import json
from datetime import datetime
import time
import os

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Mock responses for testing
MOCK_RESPONSES = {
    "hi": "Hello! I'm the Physical AI and Humanoid Robotics assistant. I can help you with questions about ROS 2 fundamentals, AI algorithms, control systems, and humanoid dynamics. What would you like to know?",
    "hello": "Hello! I'm the Physical AI and Humanoid Robotics assistant. I can help you with questions about ROS 2 fundamentals, AI algorithms, control systems, and humanoid dynamics. What would you like to know?",
    "module": "The book is organized into four modules: 1) ROS 2 Fundamentals, 2) Core AI Algorithms, 3) Control and Planning, and 4) Advanced Kinematics & Dynamics. Which module would you like to know more about?",
    "robot": "Humanoid robotics involves creating robots with human-like characteristics. This includes understanding kinematics, dynamics, control systems, and AI algorithms for locomotion and interaction. Our book covers these topics in detail.",
    "book": "The Physical AI and Humanoid Robotics book is organized into four modules covering ROS 2, AI algorithms, control systems, and dynamics. What specific topic would you like to explore?",
    "default": "I can help you with questions about Physical AI and Humanoid Robotics. The book covers ROS 2 fundamentals, AI algorithms, control systems, and dynamics. What would you like to know?"
}

@app.route('/query', methods=['POST'])
def query():
    start_time = time.time()
    data = request.json
    query_text = data.get('query', '').lower()
    query_id = f"query_{int(datetime.utcnow().timestamp())}"

    # Find a matching response or use default
    response_text = MOCK_RESPONSES.get(query_text, MOCK_RESPONSES['default'])

    # Create mock content chunks
    mock_chunks = [{
        "text": f"This is a mock response for your query: '{data.get('query', 'unknown')}'. The Physical AI and Humanoid Robotics book covers topics like ROS 2 fundamentals, AI algorithms, control systems, and dynamics.",
        "url": "https://example.com/mock",
        "page_title": "Physical AI and Humanoid Robotics",
        "section": "Mock Section",
        "score": 0.95,
        "source_url": "https://example.com/mock"
    }]

    # Simulate processing time
    time.sleep(0.1)
    processing_time = (time.time() - start_time) * 1000

    response = {
        "response_id": query_id,
        "query_id": query_id,
        "answer_text": response_text,
        "source_chunks": mock_chunks,
        "status": "success",
        "timestamp": datetime.utcnow().isoformat()
    }

    return jsonify(response)

@app.route('/', methods=['GET'])
def health():
    return jsonify({"status": "ok", "message": "RAG Agent API is running"})

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 8001))  # Changed to 8001 to avoid conflicts
    app.run(host='0.0.0.0', port=port, debug=True)