# API Contracts - Phase 1

**Note**: Phase 1 (Book Infrastructure) does not require backend API contracts.

This directory is prepared for future phases that will need API definitions:

- **Phase 3**: RAG Chatbot Core - `/api/query`, `/api/search` endpoints
- **Phase 4A**: Authentication - `/api/auth/*` endpoints
- **Phase 4B**: Personalization - `/api/user/*`, `/api/recommendations` endpoints

## Future Contract Structure

When backend APIs are introduced in later phases, contracts will be organized as:

```
contracts/
├── v1/
│   ├── chatbot.yaml     # RAG query API
│   ├── auth.yaml        # Authentication endpoints
│   └── personalization.yaml # User preferences API
└── README.md            # This file
```

Each contract will follow OpenAPI 3.1 specification with:
- Request/response schemas
- Authentication requirements
- Rate limiting specifications
- Error response formats