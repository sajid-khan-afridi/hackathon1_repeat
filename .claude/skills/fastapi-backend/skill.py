#!/usr/bin/env python3
"""
FastAPI Backend Skill
Creates REST API endpoints for RAG chatbot and user management
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import Dict, Any, Optional

# Add the backend directory to the path
backend_dir = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_dir))

def create_endpoint(endpoint_type: str, route_path: str, request_model: Dict, response_model: Dict) -> Dict[str, Any]:
    """
    Create a new endpoint based on the specified parameters
    """

    # Generate endpoint code based on type
    if endpoint_type == "rag_query":
        return create_rag_endpoint(route_path, request_model, response_model)
    elif endpoint_type == "user_auth":
        return create_auth_endpoint(route_path, request_model, response_model)
    elif endpoint_type == "personalization":
        return create_personalization_endpoint(route_path, request_model, response_model)
    else:
        raise ValueError(f"Unknown endpoint type: {endpoint_type}")

def create_rag_endpoint(route_path: str, request_model: Dict, response_model: Dict) -> Dict[str, Any]:
    """Create RAG query endpoint"""

    endpoint_code = f'''
@router.post("{route_path}", response_model={response_model.get('name', 'ChatResponse')})
async def custom_rag_query(
    request: {request_model.get('name', 'CustomChatRequest')},
    background_tasks: BackgroundTasks,
    user: dict = Depends(verify_token)
):
    """
    Custom RAG query endpoint
    """
    try:
        start_time = time.time()

        # Process query with custom parameters
        result = await rag_service.query(
            query=request.query,
            context=getattr(request, 'context', None),
            temperature=getattr(request, 'temperature', 0.7),
            max_tokens=getattr(request, 'max_tokens', 500)
        )

        response_time = time.time() - start_time

        # Create response
        response = {response_model.get('name', 'ChatResponse')}(
            answer=result["answer"],
            sources=result.get("sources", []),
            query=request.query,
            response_time=response_time,
            **{{k: getattr(request, k, None) for k in request.__dict__ if k not in ['query', 'context', 'temperature', 'max_tokens']}}
        )

        return response

    except Exception as e:
        logger.error(f"Custom RAG query error: {{str(e)}}")
        raise HTTPException(status_code=500, detail="Failed to process custom query")
'''

    return {
        "type": "rag_query",
        "route_path": route_path,
        "code": endpoint_code,
        "models": {
            "request": request_model,
            "response": response_model
        }
    }

def create_auth_endpoint(route_path: str, request_model: Dict, response_model: Dict) -> Dict[str, Any]:
    """Create authentication endpoint"""

    endpoint_code = f'''
@router.post("{route_path}", response_model={response_model.get('name', 'TokenResponse')})
async def custom_auth_endpoint(request: {request_model.get('name', 'CustomAuthRequest')}):
    """
    Custom authentication endpoint
    """
    try:
        # Custom authentication logic
        user = await auth_service.custom_authenticate(
            **request.__dict__
        )

        if not user:
            raise HTTPException(status_code=401, detail="Authentication failed")

        # Generate token
        access_token = await auth_service.create_access_token(
            data={{"sub": user["user_id"], "email": user["email"]}}
        )

        return {response_model.get('name', 'TokenResponse')}(
            access_token=access_token,
            token_type="bearer",
            expires_in=settings.access_token_expire_minutes * 60,
            user=UserResponse(**user)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Custom auth error: {{str(e)}}")
        raise HTTPException(status_code=500, detail="Authentication failed")
'''

    return {
        "type": "user_auth",
        "route_path": route_path,
        "code": endpoint_code,
        "models": {
            "request": request_model,
            "response": response_model
        }
    }

def create_personalization_endpoint(route_path: str, request_model: Dict, response_model: Dict) -> Dict[str, Any]:
    """Create personalization endpoint"""

    endpoint_code = f'''
@router.{request_model.get('method', 'post').lower()}("{route_path}", response_model={response_model.get('name', 'CustomPersonalizationResponse')})
async def custom_personalization_endpoint(
    request: {request_model.get('name', 'CustomPersonalizationRequest')} = None,
    user: dict = Depends(verify_token)
):
    """
    Custom personalization endpoint
    """
    try:
        # Custom personalization logic
        result = await analytics_service.custom_operation(
            user_id=user["user_id"],
            **(request.__dict__ if request else {{}})
        )

        return {response_model.get('name', 'CustomPersonalizationResponse')}(**result)

    except Exception as e:
        logger.error(f"Custom personalization error: {{str(e)}}")
        raise HTTPException(status_code=500, detail="Personalization operation failed")
'''

    return {
        "type": "personalization",
        "route_path": route_path,
        "code": endpoint_code,
        "models": {
            "request": request_model,
            "response": response_model
        }
    }

def generate_openapi_spec(endpoints: list) -> Dict[str, Any]:
    """Generate OpenAPI specification for custom endpoints"""

    spec = {
        "openapi": "3.0.0",
        "info": {
            "title": "RAG Chatbot Backend API",
            "version": "1.0.0",
            "description": "Custom endpoints for RAG chatbot"
        },
        "paths": {}
    }

    for endpoint in endpoints:
        path = endpoint["route_path"]
        method = endpoint["models"]["request"].get("method", "post").lower()

        if path not in spec["paths"]:
            spec["paths"][path] = {}

        spec["paths"][path][method] = {
            "summary": f"{endpoint['type']} endpoint",
            "operationId": f"{endpoint['type'].replace('_', '')}_{path.replace('/', '_')}",
            "tags": [endpoint["type"]],
            "requestBody": {
                "content": {
                    "application/json": {
                        "schema": {
                            "$ref": f"#/components/schemas/{endpoint['models']['request'].get('name', 'CustomRequest')}"
                        }
                    }
                }
            },
            "responses": {
                "200": {
                    "content": {
                        "application/json": {
                            "schema": {
                                "$ref": f"#/components/schemas/{endpoint['models']['response'].get('name', 'CustomResponse')}"
                            }
                        }
                    }
                }
            }
        }

    return spec

def main():
    parser = argparse.ArgumentParser(description="FastAPI Backend Skill")
    parser.add_argument("--endpoint-type", choices=["rag_query", "user_auth", "personalization"], required=True)
    parser.add_argument("--route-path", required=True, help="API route path")
    parser.add_argument("--request-model", required=True, help="JSON file with request model definition")
    parser.add_argument("--response-model", required=True, help="JSON file with response model definition")
    parser.add_argument("--output-dir", default="custom_endpoints", help="Output directory for generated code")

    args = parser.parse_args()

    # Load model definitions
    with open(args.request_model, 'r') as f:
        request_model = json.load(f)

    with open(args.response_model, 'r') as f:
        response_model = json.load(f)

    # Create endpoint
    endpoint = create_endpoint(args.endpoint_type, args.route_path, request_model, response_model)

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    # Write endpoint code
    endpoint_file = output_dir / f"{args.endpoint_type}_{args.route_path.strip('/')}.py"
    with open(endpoint_file, 'w') as f:
        f.write(f"""# Auto-generated endpoint - {args.endpoint_type}:{args.route_path}
# Generated on: {__import__('datetime').datetime.now()}

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
import time
import logging

# TODO: Import required models and services
# from ..models import {request_model.get('name', 'CustomRequest')}, {response_model.get('name', 'CustomResponse')}
# from ..services import rag_service, auth_service, analytics_service

logger = logging.getLogger(__name__)
router = APIRouter()

{endpoint['code']}
""")

    # Generate OpenAPI spec
    openapi_spec = generate_openapi_spec([endpoint])

    # Write OpenAPI spec
    openapi_file = output_dir / "openapi.json"
    with open(openapi_file, 'w') as f:
        json.dump(openapi_spec, f, indent=2)

    print(f"✅ Endpoint created: {endpoint_file}")
    print(f"✅ OpenAPI spec: {openapi_file}")
    print(f"📋 Add this to your router: include_router({endpoint_file.stem}.router)")

if __name__ == "__main__":
    main()