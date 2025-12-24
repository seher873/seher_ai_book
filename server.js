const express = require('express');
const path = require('path');
const { createProxyMiddleware } = require('http-proxy-middleware');
const app = express();
const PORT = process.env.PORT || 3000;

// Serve static files from the 'build' directory
app.use(express.static(path.join(__dirname, 'build')));

// Proxy API requests to the backend chat service
app.use('/api', createProxyMiddleware({
  target: process.env.CHATBOT_API_URL || 'http://localhost:8000',
  changeOrigin: true,
  pathRewrite: {
    '^/api': '', // Remove /api prefix when forwarding to backend
  },
  onProxyReq: (proxyReq, req, res) => {
    console.log(`Proxying ${req.method} ${req.url} to ${process.env.CHATBOT_API_URL || 'http://localhost:8000'}`);
  },
  onProxyRes: (proxyRes, req, res) => {
    console.log(`Received response from backend: ${proxyRes.statusCode} for ${req.url}`);
  }
}));

// Specific routes for our pages
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'index.html'));
});

app.get('/dashboard', (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'dashboard', 'index.html'));
});

app.get('/network-test', (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'network-test', 'index.html'));
});

// For other routes, serve static files if they exist, otherwise return index.html for client-side routing
app.get(/.*/, (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'index.html'));
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`🚀 Server is running on http://0.0.0.0:${PORT}`);
  console.log(`🌐 Access your site from any device on your network`);
  console.log(`📝 Images are served from /img/ directory`);
  console.log(`   - Logo: /img/logoo.png`);
  console.log(`   - Dashboard: /img/roobot.png`);
  console.log(`   - Homepage: /img/robot.png`);
  console.log(`🔗 API requests to /api will be forwarded to ${process.env.CHATBOT_API_URL || 'http://localhost:8000'}`);
});