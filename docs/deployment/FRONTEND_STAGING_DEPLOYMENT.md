# Frontend Staging Deployment Guide

**Purpose**: Deploy Docusaurus frontend to staging environment for Phase 4B UAT
**Stack**: Docusaurus (React-based static site)
**Deployment Options**: GitHub Pages staging branch, Vercel, Netlify
**Date**: 2025-12-23

---

## Prerequisites

- Frontend built successfully locally: `npm run build`
- Railway staging backend deployed and accessible
- Staging backend URL available (e.g., `https://hackathon1-staging.up.railway.app`)
- Git repository with GitHub Actions configured

---

## Option 1: Vercel Deployment (Recommended for Staging)

### Advantages
- ✅ Instant preview deployments for PRs
- ✅ Automatic HTTPS with custom domains
- ✅ Environment variable management
- ✅ Built-in analytics and performance monitoring
- ✅ Free tier sufficient for staging

### Setup Instructions

#### 1. Install Vercel CLI

```bash
npm install -g vercel
```

#### 2. Login to Vercel

```bash
vercel login
```

#### 3. Create Vercel Project

```bash
# From project root
cd "D:\GitHub Connected\hackathon1_repeat"

# Initialize Vercel project
vercel

# Answer prompts:
# ? Set up and deploy "D:\GitHub Connected\hackathon1_repeat"? [Y/n] y
# ? Which scope do you want to deploy to? [Your Name]
# ? Link to existing project? [y/N] n
# ? What's your project's name? hackathon1-staging
# ? In which directory is your code located? ./
# ? Want to override the settings? [y/N] n
```

#### 4. Configure Environment Variables

Add staging backend URL:

```bash
# Via Vercel CLI
vercel env add API_URL

# When prompted:
# ? What's the value of API_URL? https://hackathon1-staging.up.railway.app
# ? Add API_URL to which Environments? Production, Preview, Development
```

Or via Vercel Dashboard:
1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Select project: `hackathon1-staging`
3. Go to **Settings** → **Environment Variables**
4. Add:
   - Name: `API_URL`
   - Value: `https://hackathon1-staging.up.railway.app`
   - Environments: Preview, Development

#### 5. Configure Build Settings

In Vercel Dashboard → **Settings** → **Build & Development Settings**:

```
Framework Preset: Docusaurus
Build Command: npm run build
Output Directory: build
Install Command: npm install
```

#### 6. Deploy Staging Branch

```bash
# Deploy specific branch to staging
vercel --prod --branch 1-personalization-engine

# Or deploy current branch
git checkout 1-personalization-engine
vercel --prod
```

#### 7. Verify Deployment

```bash
# Vercel provides deployment URL
# Example: https://hackathon1-staging-abc123.vercel.app

# Test health
curl https://hackathon1-staging-abc123.vercel.app

# Test API connectivity
# Open browser and navigate to chatbot page
```

#### 8. Setup Custom Domain (Optional)

1. In Vercel Dashboard → **Settings** → **Domains**
2. Add domain: `staging.yourdomain.com`
3. Configure DNS CNAME to point to Vercel

#### 9. Configure CORS in Railway Backend

Update Railway staging environment variable:

```
CORS_ORIGINS=https://hackathon1-staging-abc123.vercel.app,http://localhost:3000
```

---

## Option 2: Netlify Deployment

### Advantages
- ✅ Similar features to Vercel
- ✅ Good GitHub integration
- ✅ Instant preview deploys

### Setup Instructions

#### 1. Install Netlify CLI

```bash
npm install -g netlify-cli
```

#### 2. Login to Netlify

```bash
netlify login
```

#### 3. Initialize Netlify Site

```bash
cd "D:\GitHub Connected\hackathon1_repeat"

# Initialize site
netlify init

# Answer prompts:
# ? What would you like to do? Create & configure a new site
# ? Team: [Your Team]
# ? Site name (optional): hackathon1-staging
# ? Your build command: npm run build
# ? Directory to deploy: build
```

#### 4. Configure Environment Variables

```bash
# Add staging backend URL
netlify env:set API_URL "https://hackathon1-staging.up.railway.app"
```

Or via Netlify Dashboard:
1. Go to [Netlify Dashboard](https://app.netlify.com)
2. Select site: `hackathon1-staging`
3. Go to **Site settings** → **Environment variables**
4. Add `API_URL` = `https://hackathon1-staging.up.railway.app`

#### 5. Deploy

```bash
# Deploy to production (staging)
netlify deploy --prod

# Or specify directory
netlify deploy --prod --dir=build
```

#### 6. Configure CORS

Update Railway staging:
```
CORS_ORIGINS=https://hackathon1-staging.netlify.app,http://localhost:3000
```

---

## Option 3: GitHub Pages Staging Branch

### Advantages
- ✅ Free hosting on GitHub
- ✅ Integrated with repository
- ✅ No external accounts needed

### Disadvantages
- ❌ No environment variable support (requires build-time injection)
- ❌ Slower deployment
- ❌ Less flexible than Vercel/Netlify

### Setup Instructions

#### 1. Create Staging Branch Workflow

Create: `.github/workflows/deploy-staging.yml`

```yaml
name: Deploy to GitHub Pages Staging

on:
  push:
    branches: [1-personalization-engine]
  workflow_dispatch:

permissions:
  contents: write
  pages: write
  id-token: write

jobs:
  build-staging:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: "20"
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build website (staging)
        run: npm run build
        env:
          API_URL: https://hackathon1-staging.up.railway.app

      - name: Deploy to gh-pages-staging branch
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          publish_branch: gh-pages-staging
          cname: staging.yourdomain.com  # Optional: if using custom domain
```

#### 2. Enable GitHub Pages for Staging Branch

1. Go to GitHub repository **Settings** → **Pages**
2. Source: Deploy from branch
3. Branch: `gh-pages-staging` / `root`
4. Save

#### 3. Access Staging Site

- URL: `https://sajid-khan-afridi.github.io/hackathon1_repeat/` (from gh-pages-staging)
- Note: May need custom subdomain to differentiate from production

#### 4. Configure Build-Time Environment Variable

Update `docusaurus.config.ts`:

```typescript
const config: Config = {
  // ...
  customFields: {
    apiUrl: process.env.API_URL || 'https://hackathon1repeat-production.up.railway.app',
  },
  // ...
};
```

Build with staging URL:
```bash
API_URL=https://hackathon1-staging.up.railway.app npm run build
```

---

## Frontend Configuration for Staging

### Update docusaurus.config.ts

Ensure environment variable is read correctly:

```typescript
// docusaurus.config.ts
const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  url: process.env.VERCEL_URL
    ? `https://${process.env.VERCEL_URL}`
    : 'https://sajid-khan-afridi.github.io',
  baseUrl: process.env.VERCEL_URL ? '/' : '/hackathon1_repeat/',

  customFields: {
    // Use environment variable for API URL (staging vs production)
    apiUrl: process.env.API_URL || 'https://hackathon1repeat-production.up.railway.app',
  },
};
```

### Update Frontend Code to Use Custom Field

Ensure components use the configured API URL:

```typescript
// src/services/personalization-api.ts
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export function useApiUrl(): string {
  const { siteConfig } = useDocusaurusContext();
  return siteConfig.customFields.apiUrl as string;
}

// Usage in components
const apiUrl = useApiUrl();
const response = await fetch(`${apiUrl}/api/v1/recommendations`);
```

---

## Build Script for Staging

Create staging-specific build script in `package.json`:

```json
{
  "scripts": {
    "build": "docusaurus build",
    "build:staging": "cross-env API_URL=https://hackathon1-staging.up.railway.app docusaurus build",
    "build:production": "cross-env API_URL=https://hackathon1repeat-production.up.railway.app docusaurus build"
  }
}
```

Install `cross-env` for cross-platform env vars:
```bash
npm install --save-dev cross-env
```

Build for staging:
```bash
npm run build:staging
```

---

## Testing Frontend Staging Deployment

### 1. Verify Build Locally

```bash
# Build with staging API URL
npm run build:staging

# Serve locally
npm run serve

# Visit http://localhost:3000
# Open browser dev tools → Network tab
# Verify API calls go to staging backend
```

### 2. Check API URL in Browser

Open browser console:
```javascript
// Check configured API URL
const siteConfig = window.docusaurus.siteConfig;
console.log('API URL:', siteConfig.customFields.apiUrl);
// Expected: https://hackathon1-staging.up.railway.app
```

### 3. Test Personalization Features

1. **Navigate to Profile Page**: `/profile`
2. **Check Skill Level Badge**: Should load from staging API
3. **Check Recommendations**: Should fetch from staging
4. **Check Progress Tracking**: Should save to staging database

### 4. Verify CORS

Check browser console for CORS errors:
```
Access to fetch at 'https://hackathon1-staging.up.railway.app/api/v1/recommendations'
from origin 'https://hackathon1-staging.vercel.app' has been blocked by CORS policy
```

If CORS error occurs:
- Update Railway staging `CORS_ORIGINS` variable
- Redeploy Railway backend
- Clear browser cache

---

## CI/CD Integration

### Automatic Staging Deployment on Push

#### For Vercel

Vercel automatically deploys on push to connected branches:

1. Go to Vercel Dashboard → **Settings** → **Git**
2. Connect GitHub repository
3. Production Branch: `main`
4. Enable **Preview Deployments**: `1-personalization-engine`

Every push to `1-personalization-engine` triggers staging deployment.

#### For Netlify

1. Netlify Dashboard → **Site settings** → **Build & deploy**
2. Connect GitHub repository
3. Production Branch: `main`
4. Enable **Branch deploys**: `1-personalization-engine`

#### For GitHub Pages

Use the workflow created in Option 3 above. Automatically deploys on push to feature branch.

---

## Environment-Specific Configuration Summary

| Environment | Frontend URL | Backend URL | Branch | Deployment |
|-------------|-------------|-------------|--------|------------|
| **Development** | http://localhost:3000 | http://localhost:8000 | any | local |
| **Staging** | https://hackathon1-staging.vercel.app | https://hackathon1-staging.up.railway.app | 1-personalization-engine | Vercel/Netlify |
| **Production** | https://sajid-khan-afridi.github.io/hackathon1_repeat/ | https://hackathon1repeat-production.up.railway.app | main | GitHub Pages |

---

## Troubleshooting

### Build Fails: "API_URL is not defined"

**Cause**: Environment variable not set during build

**Fix**:
```bash
# Set before build
export API_URL=https://hackathon1-staging.up.railway.app
npm run build

# Or use build:staging script
npm run build:staging
```

### API Calls Return 404

**Cause**: Wrong API URL or backend not deployed

**Fix**:
1. Check browser console for actual URL being called
2. Verify backend is accessible: `curl https://hackathon1-staging.up.railway.app/health`
3. Check `docusaurus.config.ts` customFields.apiUrl value

### CORS Error on Staging

**Cause**: Railway backend CORS_ORIGINS doesn't include staging frontend URL

**Fix**:
1. Get exact staging frontend URL (including https://)
2. Update Railway staging `CORS_ORIGINS`:
   ```
   CORS_ORIGINS=https://hackathon1-staging.vercel.app,http://localhost:3000
   ```
3. Redeploy Railway backend
4. Hard refresh browser (Ctrl+Shift+R)

### Personalization Features Not Working

**Cause**: Missing authentication or API endpoint errors

**Fix**:
1. Check browser console for API errors
2. Verify user is authenticated (OAuth working)
3. Check Railway logs for backend errors
4. Test endpoints directly:
   ```bash
   curl https://hackathon1-staging.up.railway.app/api/v1/skill-level
   ```

---

## Post-Deployment Checklist

After frontend staging deployment:

- ✅ Staging URL accessible
- ✅ API URL configured correctly (check browser console)
- ✅ CORS allowing requests from staging frontend
- ✅ Homepage loads without errors
- ✅ Chatbot page accessible (`/chatbot`)
- ✅ Profile page accessible (`/profile`)
- ✅ OAuth login working (Google/GitHub)
- ✅ Personalization features loading:
  - ✅ Skill level badge displays
  - ✅ Recommendations load
  - ✅ Progress tracking works
- ✅ No console errors
- ✅ No CORS errors
- ✅ API calls going to staging backend (verify in Network tab)

Proceed to: **Smoke Testing** (see `SMOKE_TEST_CHECKLIST.md`)

---

## Next Steps

1. ✅ **Complete**: Frontend deployed to staging
2. ⏭️ **Next**: Run smoke tests (see `SMOKE_TEST_CHECKLIST.md`)
3. ⏭️ **Next**: Execute UAT (see `UAT_TEST_PLAN.md`)
4. ⏭️ **Next**: Document UAT results
5. ⏭️ **Next**: Deploy to production (after UAT approval)

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Maintained By**: DevOps Team
