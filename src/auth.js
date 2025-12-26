const { betterAuth } = require("better-auth");

// Create the auth instance with email/password authentication
const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./sqlite.db",
  },
  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },
  // Session configuration
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60, // 24 hours
  },
  // Account configuration
  account: {
    accountLinking: {
      enabled: true,
    },
  },
  // User configuration
  user: {
    data: [
      {
        key: "name",
        type: "string",
        required: true,
      }
    ]
  },
  // App name for email templates
  app: {
    name: "Physical AI Textbook",
  },
  // Secret for signing tokens (use a strong secret in production)
  secret: process.env.BETTER_AUTH_SECRET || "your-development-secret-change-in-production",
});

module.exports = { auth };