// models/user.js

const { Pool } = require('pg');
const bcrypt = require('bcryptjs');
const crypto = require('crypto');

// Helper function to validate UUID format
const isUUID = (id) => /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i.test(id);

// Set up the connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL + "?sslmode=require",
});

// User model with methods for registration and management
const User = {
    // Find user by ID (UUID)
    findOne: async (id) => {
      if (!isUUID(id)) {
        throw new Error(`Invalid UUID format: ${id}`); // Prevent invalid queries
      }
      const res = await pool.query(
        'SELECT * FROM "acorn-users-db".users WHERE id = $1',
        [id]
      );
      if (res.rows.length === 0) throw new Error(`User not found with ID: ${id}`);
      return res.rows[0]; // Returns a single user object
    },
  
    // Find user by email
    findByEmail: async (email) => {
      const res = await pool.query(
        'SELECT * FROM "acorn-users-db".users WHERE email = $1',
        [email]
      );
      if (res.rows.length === 0) throw new Error(`User not found with email: ${email}`);
      return res.rows[0]; // Returns a single user object
    },
  
    // Find user by either ID or email
    findByIdOrEmail: async (identifier) => {
      if (isUUID(identifier)) {
        return await User.findOne(identifier); // Find by UUID
      } else {
        return await User.findByEmail(identifier); // Find by email
      }
    },
  
    // Register a new user
    create: async (userData) => {
      const { email, id, password } = userData;
      const salt = await bcrypt.genSalt(10);
      const hashedPassword = await bcrypt.hash(password, salt);
  
      const result = await pool.query(
        'INSERT INTO "acorn-users-db".users (email, id, password_hash) VALUES ($1, $2, $3) RETURNING id, email',
        [email, id, hashedPassword]
      );
  
      if (result.rows.length === 0) throw new Error('Failed to create new user');
      return result.rows[0]; // Return the newly created user
    },
  
    // Verify user email
    verifyEmail: async (id, verificationCode) => {
      const res = await pool.query(
        `SELECT * FROM "acorn-users-db".email_verifications 
         WHERE user_id = $1 AND verification_code = $2 AND expires_at > NOW()`,
        [id, verificationCode]
      );
      if (res.rows.length === 0) throw new Error('Invalid or expired verification code');
      return res.rows[0]; // Returns a verification record
    },
  
    // Create email verification record
    createEmailVerification: async (id) => {
      const verificationCode = crypto.randomBytes(16).toString('hex');
      const expiresAt = new Date(Date.now() + 3600000); // 1 hour expiration
  
      const result = await pool.query(
        `INSERT INTO "acorn-users-db".email_verifications 
         (user_id, verification_code, expires_at) VALUES ($1, $2, $3) RETURNING id`,
        [id, verificationCode, expiresAt]
      );
  
      if (result.rows.length === 0) throw new Error('Failed to create email verification');
      return result.rows[0]; // Return newly created verification record
    },
  
    // Create password reset token
    createPasswordReset: async (id) => {
      const resetToken = crypto.randomBytes(16).toString('hex');
      const expiresAt = new Date(Date.now() + 3600000); // 1 hour expiration
  
      const result = await pool.query(
        `INSERT INTO "acorn-users-db".password_resets 
         (user_id, reset_token, expires_at) VALUES ($1, $2, $3) RETURNING id`,
        [id, resetToken, expiresAt]
      );
  
      if (result.rows.length === 0) throw new Error('Failed to create password reset');
      return result.rows[0]; // Return newly created password reset record
    },
  
    // Find password reset record
    findPasswordReset: async (resetToken) => {
      const res = await pool.query(
        `SELECT * FROM "acorn-users-db".password_resets 
         WHERE reset_token = $1 AND expires_at > NOW()`,
        [resetToken]
      );
      if (res.rows.length === 0) throw new Error('Invalid or expired reset token');
      return res.rows[0]; // Returns a reset record
    }
  };
  
  module.exports = User;