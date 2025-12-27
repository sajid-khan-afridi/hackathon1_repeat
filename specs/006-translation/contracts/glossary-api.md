# Glossary API Contract

**Feature**: 006-translation
**Date**: 2025-12-27
**Type**: Static JSON (no REST API)

## Overview

The glossary is served as a static JSON file at build time. No runtime API is required.

## Static Endpoint

### GET /data/glossary.json

Returns the complete bilingual glossary.

**Response**: `application/json`

```json
{
  "version": "1.0.0",
  "updatedAt": "2025-12-27T10:30:00Z",
  "termCount": 50,
  "categories": [
    "Robotics Frameworks",
    "Programming Languages",
    "Hardware Components",
    "AI/ML Concepts",
    "Mathematics",
    "Simulation"
  ],
  "terms": [
    {
      "id": "ros-2",
      "english": "ROS 2",
      "urduTransliteration": "آر او ایس ٹو",
      "definition": "Robot Operating System 2 - the next-generation robotics middleware",
      "definitionUrdu": "روبوٹ آپریٹنگ سسٹم 2 - اگلی نسل کا روبوٹکس مڈل ویئر",
      "category": "Robotics Frameworks",
      "relatedTerms": ["roscore", "ros-node"]
    }
  ]
}
```

## Client Usage

### React Hook

```typescript
// src/hooks/useGlossary.ts
import glossaryData from '@site/src/data/glossary.json';

export function useGlossary() {
  return {
    terms: glossaryData.terms,
    categories: glossaryData.categories,
    getTermById: (id: string) => glossaryData.terms.find(t => t.id === id),
    getTermsByCategory: (category: string) =>
      glossaryData.terms.filter(t => t.category === category),
    searchTerms: (query: string) =>
      glossaryData.terms.filter(t =>
        t.english.toLowerCase().includes(query.toLowerCase()) ||
        t.definition.toLowerCase().includes(query.toLowerCase())
      ),
  };
}
```

### GlossaryTooltip Component

```typescript
// src/components/GlossaryTooltip/index.tsx
interface GlossaryTooltipProps {
  termId: string;
  children: React.ReactNode;
}

export function GlossaryTooltip({ termId, children }: GlossaryTooltipProps) {
  const { getTermById } = useGlossary();
  const term = getTermById(termId);

  if (!term) return <>{children}</>;

  return (
    <Tooltip content={`${term.english}: ${term.definition}`}>
      <span className="glossary-term">{children}</span>
    </Tooltip>
  );
}
```

## Language Preference API

For authenticated users, language preference is stored in the user profile.

### PATCH /api/user/preferences

Updates user's language preference.

**Request Body**:
```json
{
  "preferredLanguage": "ur"
}
```

**Response**: `200 OK`
```json
{
  "success": true,
  "preferredLanguage": "ur"
}
```

**Error Responses**:
- `400 Bad Request`: Invalid language code (must be "en" or "ur")
- `401 Unauthorized`: User not authenticated

## Notes

- Glossary data is bundled at build time (no runtime API calls)
- Language preference sync requires authenticated session
- Guest users use localStorage only (no API)
