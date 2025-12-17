# Frontend Deployment Guide - GitHub Pages

## Quick Deployment Steps

### Step 1: Update Railway CORS Configuration ⚠️ CRITICAL

Before deploying frontend, you **MUST** update CORS on Railway backend:

1. Go to **Railway Dashboard** → Your Project → **Variables**
2. Find `CORS_ORIGINS` variable
3. Update it to include GitHub Pages domain:

```bash
CORS_ORIGINS=http://localhost:3000,http://localhost:8000,https://sajid-khan-afridi.github.io
```

4. **Save** and wait for Railway to redeploy (~30 seconds)

Without this, the frontend **cannot** communicate with the backend! ❌

---

### Step 2: Deploy to GitHub Pages

Run the deployment command:

```bash
npm run deploy
```

This will:
- Build the Docusaurus site (`npm run build`)
- Push to `gh-pages` branch
- Deploy to GitHub Pages automatically

**Deployment time**: ~2-3 minutes

---

### Step 3: Enable GitHub Pages (First Time Only)

If this is your first deployment:

1. Go to **GitHub** → Repository → **Settings** → **Pages**
2. Under "Source", select:
   - **Branch**: `gh-pages`
   - **Folder**: `/ (root)`
3. Click **Save**
4. Wait 1-2 minutes for GitHub to publish

---

### Step 4: Access Your Site

Your site will be live at:

**URL**: https://sajid-khan-afridi.github.io/hackathon1_repeat/

It may take a few minutes for DNS to propagate.

---

## Post-Deployment Verification

### Test the Chatbot Widget

1. Visit your deployed site
2. Navigate to **🤖 AI Assistant** page
3. Try asking a question: "What is ROS?"
4. Verify:
   - ✅ Message sends successfully
   - ✅ Response appears with sources
   - ✅ No CORS errors in browser console (F12 → Console)

### Check Browser Console

Press **F12** → **Console** tab and verify:

```
[Chatbot] API URL configured: https://hackathon1repeat-production.up.railway.app
```

If you see CORS errors like:
```
Access to fetch at 'https://...' has been blocked by CORS policy
```

**Fix**: Go back to Step 1 and update CORS_ORIGINS in Railway.

---

## Troubleshooting

### Issue: CORS Error

**Symptom**:
```
Access to fetch at 'https://hackathon1repeat-production.up.railway.app/api/v1/query'
from origin 'https://sajid-khan-afridi.github.io' has been blocked by CORS policy
```

**Solution**:
1. Check Railway → Variables → CORS_ORIGINS
2. Ensure it includes: `https://sajid-khan-afridi.github.io`
3. Save and wait for redeploy
4. Hard refresh your frontend (Ctrl + Shift + R)

---

### Issue: 404 Page Not Found

**Symptom**: All pages show 404 except homepage

**Solution**:
Check `docusaurus.config.ts`:
```typescript
url: 'https://sajid-khan-afridi.github.io',
baseUrl: '/hackathon1_repeat/',  // Must match repo name
```

---

### Issue: Chatbot Not Working

**Check these in order**:

1. **Is backend healthy?**
   ```bash
   curl https://hackathon1repeat-production.up.railway.app/api/v1/health
   ```
   Should return: `{"status": "healthy", ...}`

2. **Is CORS configured?**
   - Check Railway Variables → CORS_ORIGINS
   - Must include GitHub Pages domain

3. **Is frontend pointing to correct backend?**
   - Check browser console (F12)
   - Should show: `[Chatbot] API URL configured: https://hackathon1repeat-production.up.railway.app`

4. **Network errors?**
   - F12 → Network tab
   - Try sending a message
   - Check if request reaches backend (should show 200 OK)

---

## Custom Domain (Optional)

If you want a custom domain like `robotics.yourdomain.com`:

### For GitHub Pages:

1. Buy a domain (Namecheap, GoDaddy, etc.)
2. Add `CNAME` file to `static/` folder:
   ```
   robotics.yourdomain.com
   ```
3. Configure DNS:
   - Add CNAME record: `robotics` → `sajid-khan-afridi.github.io`
4. Update `docusaurus.config.ts`:
   ```typescript
   url: 'https://robotics.yourdomain.com',
   baseUrl: '/',
   ```
5. Redeploy: `npm run deploy`
6. Update Railway CORS_ORIGINS to include new domain

---

## Performance Optimization

### Enable Service Worker (PWA)

Add to `docusaurus.config.ts`:

```typescript
presets: [
  [
    'classic',
    {
      // ... existing config
      gtag: {
        trackingID: 'G-YOUR-TRACKING-ID',
      },
      sitemap: {
        changefreq: 'weekly',
        priority: 0.5,
      },
    },
  ],
],
```

### Add Analytics

Install Google Analytics:

```bash
npm install @docusaurus/plugin-google-gtag
```

Configure in `docusaurus.config.ts`:

```typescript
plugins: [
  [
    '@docusaurus/plugin-google-gtag',
    {
      trackingID: 'G-YOUR-TRACKING-ID',
      anonymizeIP: true,
    },
  ],
],
```

---

## Continuous Deployment

### Auto-deploy on Push

GitHub Pages will **NOT** auto-deploy when you push to `main`.

To enable auto-deploy, add GitHub Action:

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

Now every push to `main` will auto-deploy! 🚀

---

## Deployment Checklist

Before deploying, ensure:

- [x] Backend deployed to Railway and healthy
- [x] Railway CORS_ORIGINS includes GitHub Pages domain
- [x] Frontend API URL configured in `chatbotConfig.ts`
- [x] Docusaurus config has correct `url` and `baseUrl`
- [ ] Run `npm run deploy` to deploy
- [ ] Enable GitHub Pages in repo settings
- [ ] Test chatbot on deployed site
- [ ] Check browser console for errors
- [ ] Verify no CORS errors

---

## Cost & Maintenance

### GitHub Pages Limits

- **Free** for public repositories
- **Soft limit**: 100GB bandwidth/month
- **Build time**: 10 minutes max
- **Repository size**: 1GB recommended max

For high traffic (>100GB/month), consider:
- **Vercel** (100GB/month free)
- **Netlify** (100GB/month free)
- **Cloudflare Pages** (Unlimited free)

### Updates

To update the site:

```bash
# Make changes to docs or code
git add .
git commit -m "Update content"
git push origin main

# Deploy updated version
npm run deploy
```

---

## Next Steps

After successful deployment:

1. ✅ Share your site: `https://sajid-khan-afridi.github.io/hackathon1_repeat/`
2. 📊 Set up analytics (Google Analytics)
3. 🔍 Submit to search engines (Google Search Console)
4. 📱 Test on mobile devices
5. ♿ Run accessibility audit (Lighthouse)
6. 🚀 Consider custom domain for professional look

---

**Your URLs**:
- **Frontend**: https://sajid-khan-afridi.github.io/hackathon1_repeat/
- **Backend**: https://hackathon1repeat-production.up.railway.app
- **API Docs**: https://hackathon1repeat-production.up.railway.app/docs

Happy deploying! 🎉
