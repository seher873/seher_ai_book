const express = require("express");
const { toNodeHandler, fromNodeHeaders } = require("better-auth/node");
const { auth } = require("../src/auth.js");

const router = express.Router();

// Mount Better Auth handler for all auth routes
router.all("/api/auth/*", (req, res) => {
  // Ensure we're not parsing JSON before the Better Auth handler
  if (req.body && Object.keys(req.body).length === 0 && req.headers['content-type'] && req.headers['content-type'].includes('application/json')) {
    // If the body is empty but content-type is JSON, we need to handle it differently
    // This is to avoid issues with express.json() middleware
    let data = '';
    req.on('data', chunk => {
      data += chunk;
    });
    req.on('end', () => {
      if (data) {
        req.body = JSON.parse(data);
      }
      toNodeHandler(auth)(req, res);
    });
  } else {
    toNodeHandler(auth)(req, res);
  }
});

// Endpoint to get current user session
router.get("/api/auth/me", async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: fromNodeHeaders(req.headers),
    });

    if (session) {
      res.json({ user: session.user, session });
    } else {
      res.status(401).json({ error: "Not authenticated" });
    }
  } catch (error) {
    console.error("Error getting session:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

// Endpoint to sign out
router.post("/api/auth/signout", async (req, res) => {
  try {
    const result = await auth.api.signOut({
      headers: fromNodeHeaders(req.headers),
      body: {},
    });

    res.json(result);
  } catch (error) {
    console.error("Error signing out:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

module.exports = router;