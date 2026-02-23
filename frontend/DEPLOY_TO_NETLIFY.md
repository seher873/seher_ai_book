# Deploy Frontend to Netlify

## ✅ Configuration Complete!

Your frontend is now configured to connect to the Hugging Face backend at:
- **Backend URL**: https://sehrkhan873-robotic-text-book.hf.space
- **API Proxy**: `/api/*` → `https://sehrkhan873-robotic-text-book.hf.space/*`

---

## 🚀 Deploy to Netlify

### Option 1: Connect GitHub Repository (Recommended)

#### Step 1: Push Changes to GitHub

```bash
cd /mnt/c/Users/user/Desktop/text-book/seher_ai_book/frontend
git add netlify.toml
git commit -m "fix: update backend API URL to Hugging Face Space"
git push origin main
```

#### Step 2: Connect to Netlify

1. Go to: https://app.netlify.com/
2. Click **"Add new site"** → **"Import an existing project"**
3. Choose **GitHub**
4. Authorize Netlify to access your GitHub
5. Select repository: **`seher873/seher_ai_book`**
6. Configure build settings:
   - **Base directory**: `frontend`
   - **Build command**: `npm run build`
   - **Publish directory**: `frontend/build`
7. Click **"Deploy site"**

#### Step 3: Set Custom Domain (Optional)

If you want to use `seher-robotic-book.netlify.app`:
1. Go to **Site settings** → **Domain management**
2. Click **"Add custom domain"**
3. Enter: `seher-robotic-book.netlify.app`
4. Click **Verify** and **Save**

---

### Option 2: Manual Deploy via Netlify CLI

#### Install Netlify CLI

```bash
npm install -g netlify-cli
```

#### Login to Netlify

```bash
netlify login
```

#### Deploy

```bash
cd frontend
npm install
npm run build
netlify deploy --prod --dir=build --site=YOUR_SITE_ID
```

---

## 🔧 Netlify Configuration

### Build Settings

| Setting | Value |
|---------|-------|
| **Base directory** | `frontend` |
| **Build command** | `npm run build` |
| **Publish directory** | `frontend/build` |
| **Node version** | 18 |

### Environment Variables

No environment variables needed for frontend! The API proxy is configured in `netlify.toml`.

### Redirect Rules

The `netlify.toml` includes:
- **API Proxy**: `/api/chat` → `https://sehrkhan873-robotic-text-book.hf.space/chat`
- **SPA Fallback**: All routes serve `index.html` for client-side routing
- **Security Headers**: X-Frame-Options, X-Content-Type-Options, etc.
- **Cache Control**: Static assets cached for 1 year

---

## ✅ Verify Deployment

After deployment, test the chatbot:

1. **Visit your site**: https://seher-robotic-book.netlify.app
2. **Click the chatbot icon** (bottom-right corner)
3. **Ask a question**: "What is ROS2?"
4. **Verify response** comes from the backend

### Test API Proxy

```bash
curl https://seher-robotic-book.netlify.app/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "RAG Chatbot Backend",
  "version": "1.0.0"
}
```

---

## 🔍 Troubleshooting

### Chatbot Not Working

1. **Check browser console** for errors (F12 → Console)
2. **Verify API proxy** is working:
   ```bash
   curl https://seher-robotic-book.netlify.app/api/health
   ```
3. **Check Netlify logs**: Site settings → Deploy log

### Build Fails

1. **Check Node version**: Netlify uses Node 18 by default
2. **Clear cache**: Netlify → Deploys → Clear cache and deploy site
3. **Check dependencies**: Run `npm install` locally first

### CORS Errors

The API proxy in `netlify.toml` should handle CORS. If you see CORS errors:
1. Verify the redirect rule is active
2. Check backend CORS settings (should allow all origins)

---

## 📊 Site URLs

| Environment | URL |
|-------------|-----|
| **Production** | https://seher-robotic-book.netlify.app |
| **Deploy Preview** | `https://deploy-preview-PR_NUMBER--seher-robotic-book.netlify.app` |
| **Branch Deploy** | `https://branch-name--seher-robotic-book.netlify.app` |

---

## 🎯 Next Steps

1. ✅ Push `netlify.toml` changes to GitHub
2. ✅ Connect repository to Netlify (if not already connected)
3. ✅ Trigger deployment
4. ✅ Test chatbot functionality
5. ✅ Set up custom domain (optional)

---

## 📝 Notes

- **Automatic Deploys**: Netlify will auto-deploy on every push to `main` branch
- **Deploy Previews**: Automatic for pull requests
- **SSL Certificate**: Automatically provisioned by Netlify
- **CDN**: Global CDN included free

---

**Your frontend + backend integration is ready!** 🚀
