# RAG Chatbot Testing Checklist

## Pre-Testing Setup

### Environment Verification
- [ ] .env file created with all required variables
- [ ] PostgreSQL database running and accessible
- [ ] Qdrant vector store running and accessible
- [ ] OpenAI API key valid and has quota
- [ ] Backend server running on correct port

### Database Initialization
- [ ] Database migrations applied successfully
- [ ] Qdrant collection created
- [ ] Sample data populated (run `python backend/populate_data.py --sample`)

## Functional Testing

### User Story 1: Q&A with Citations and Confidence Scores
- [ ] Ask a question about robotics
  - Expected: Response with content, citations, and confidence score
- [ ] Verify citations include relevant sources
  - Check: Source module, chapter, and content displayed
- [ ] Test confidence indicator
  - Check: Color coding (green >80%, yellow 60-80%, red <60%)
- [ ] Verify source citations are clickable (if implemented)
  - Expected: Clicking source shows original content

### User Story 2: Conversation Continuity
- [ ] Start a conversation
  - Ask: "What is forward kinematics?"
- [ ] Ask follow-up question
  - Ask: "How does that relate to inverse kinematics?"
  - Expected: System understands context from previous messages
- [ ] Check session persistence
  - Refresh page or reopen widget
  - Expected: Conversation history restored
- [ ] Verify session management
  - Check: Session ID displayed in logs
  - Check: Messages stored in database

### User Story 3: Module Filtering
- [ ] Test without module filter
  - Ask general question
  - Expected: Search across all modules
- [ ] Apply module filter
  - Select specific module(s)
  - Ask question
  - Expected: Results filtered to selected modules
- [ ] Test multi-module selection
  - Select multiple modules
  - Verify results come from any selected module
- [ ] Test module filter persistence
  - Change filter, send message, check if filter maintained

### User Story 4: Off-topic Detection
- [ ] Ask on-topic question
  - Example: "What are PID controllers?"
  - Expected: Normal response with citations
- [ ] Ask off-topic question
  - Example: "What's the weather like?"
  - Expected: Helpful message about robotics focus
- [ ] Test edge cases
  - Question partially on-topic
  - Expected: Attempt to relate to robotics if possible
- [ ] Verify suggestions provided
  - Check: List of robotics topics suggested

### User Story 5: Rate Limiting (Backend Ready)
- [ ] Verify rate limiting headers
  - Check response headers for rate limit info
- [ ] Test rate limit enforcement
  - Send rapid requests (>30 per minute)
  - Expected: 429 Too Many Requests error
- [ ] Check rate limit reset behavior
  - Wait for reset period
  - Expected: Can make requests again

## Performance Testing

### Response Times
- [ ] Measure initial response time
  - Target: <2 seconds for first response
- [ ] Test streaming performance
  - Expected: Smooth streaming without long pauses
- [ ] Check concurrent request handling
  - Multiple simultaneous users
  - Expected: All requests handled without errors

### Resource Usage
- [ ] Monitor memory usage
  - Expected: No memory leaks over extended use
- [ ] Check database connections
  - Verify connection pooling works
  - Expected: No connection exhaustion
- [ ] Monitor OpenAI API usage
  - Check token usage per request
  - Expected: Reasonable token consumption

## UI/UX Testing

### Responsive Design
- [ ] Desktop view (>1024px)
  - Expected: Full chat interface visible
- [ ] Tablet view (768-1024px)
  - Expected: Adaptive layout
- [ ] Mobile view (<768px)
  - Expected: Full-screen or compact view
- [ ] Input field accessibility
  - Expected: Easy to type and submit on all devices

### Accessibility (WCAG 2.1 AA)
- [ ] Keyboard navigation
  - Tab through all interactive elements
  - Expected: Logical tab order
- [ ] Screen reader compatibility
  - Test with screen reader
  - Expected: All content announced
- [ ] Color contrast
  - Check text/background contrast
  - Expected: Minimum 4.5:1 ratio
- [ ] ARIA labels
  - Verify all interactive elements have labels
  - Expected: Descriptive ARIA attributes

### Error Handling
- [ ] Network error handling
  - Disconnect network during query
  - Expected: User-friendly error message
- [ ] Server error handling
  - Trigger 500 error
  - Expected: Error notification to user
- [ ] Input validation
  - Enter extremely long message
  - Expected: Character limit enforced

## Integration Testing

### API Endpoints
- [ ] Health check endpoints
  - `/health`: Returns OK
  - `/health/database`: Database status
  - `/health/vector-store`: Qdrant status
- [ ] Chat endpoint
  - POST `/api/chat`: Returns streaming response
- [ ] Session endpoints
  - GET `/api/sessions`: Returns user sessions
  - DELETE `/api/sessions/{id}`: Deletes session

### Frontend Integration
- [ ] Widget renders correctly
  - Expected: All components display properly
- [ ] EventSource connection
  - Expected: Connection established for streaming
- [ ] Error boundaries
  - Trigger JavaScript error
  - Expected: Graceful fallback

## Security Testing

### Input Validation
- [ ] SQL injection attempts
  - Input: `'; DROP TABLE users; --`
  - Expected: Rejected or sanitized
- [ ] XSS attempts
  - Input: `<script>alert('xss')</script>`
  - Expected: Not executed, displayed as text
- [ ] Large payloads
  - Send very large message
  - Expected: Size limit enforced

### Authentication/Authorization
- [ ] Unauthenticated requests
  - Access without token (if implemented)
  - Expected: 401 Unauthorized
- [ ] CORS configuration
  - Request from unauthorized origin
  - Expected: Rejected by browser

## Data Quality Testing

### Vector Search Accuracy
- [ ] Test semantic search
  - Ask: "How do robots move?"
  - Expected: Results about locomotion, actuators
- [ ] Test synonym handling
  - Ask: "What are robot arms?"
  - Expected: Results about manipulators
- [ ] Test concept relationships
  - Ask: "What's the difference between 2DOF and 6DOF?"
  - Expected: Accurate comparison

### Content Coverage
- [ ] All modules indexed
  - Verify: Content from each module is searchable
- [ ] Edge cases
  - Search for very specific topics
  - Expected: Relevant results if content exists

## End-to-End Scenarios

### Student Workflow
1. Student opens chatbot
2. Asks: "What is ROS?"
3. Follows up: "How do I install it?"
4. Filters to "software" module
5. Asks: "What publishers can I create?"
6. Closes and reopens
7. Continues conversation

### Teaching Assistant Workflow
1. TA opens chatbot
2. Asks: "What's the best way to explain DH parameters?"
3. Gets explanation with citations
4. Asks for specific examples
5. Uses responses to prepare teaching material

## Load Testing

### Concurrent Users
- [ ] 10 concurrent users
  - Expected: All users get timely responses
- [ ] 50 concurrent users
  - Expected: System remains responsive
- [ ] Stress test
  - Gradually increase load
  - Document breaking point

## Post-Testing

### Documentation Updates
- [ ] Update API documentation with any changes
- [ ] Record performance benchmarks
- [ ] Document any limitations found

### Production Readiness Checklist
- [ ] All tests passing
- [ ] Performance within acceptable limits
- [ ] Security scan completed
- [ ] Logging configured for production
- [ ] Monitoring and alerting set up
- [ ] Backup procedures documented

## Test Results Summary

| Test Category | Pass/Fail | Notes |
|---------------|-----------|-------|
| Functional Tests | | |
| Performance Tests | | |
| UI/UX Tests | | |
| Integration Tests | | |
| Security Tests | | |
| Data Quality | | |
| Load Tests | | |

## Issues Found

| Issue | Severity | Description | Resolution |
|-------|----------|-------------|------------|
| | | | |

## Recommendations

1. Prioritize fixing any Critical or High severity issues
2. Document any known limitations
3. Plan follow-up testing for addressed issues
4. Consider automated testing for regression prevention