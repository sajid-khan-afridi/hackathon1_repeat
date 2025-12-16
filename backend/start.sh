#!/bin/bash
# Startup script for Railway deployment
# Handles PORT environment variable correctly and runs migrations

# Use PORT from environment or default to 8000
PORT=${PORT:-8000}

echo "===================="
echo "Running database migrations..."
echo "===================="
python run_migrations.py

if [ $? -ne 0 ]; then
    echo "ERROR: Database migrations failed!"
    exit 1
fi

echo ""
echo "===================="
echo "Starting uvicorn on port $PORT"
echo "===================="
exec uvicorn app.main:app --host 0.0.0.0 --port "$PORT"
