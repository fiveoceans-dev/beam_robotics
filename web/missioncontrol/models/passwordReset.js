const { Pool } = require('pg');

// Set up the connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false },
});

// PasswordReset model
const PasswordReset = {
  // Find password reset by resetToken
  findByToken: async (resetToken) => {
    const res = await pool.query(
      'SELECT * FROM "acorn-users-db".password_resets WHERE reset_token = $1 AND expires_at > NOW()',
      [resetToken]
    );
    return res.rows[0];
  },

  // Create a new password reset record
  create: async (userId, resetToken) => {
    const expiresAt = new Date(Date.now() + 3600000); // 1 hour expiration time
    const result = await pool.query(
      'INSERT INTO "acorn-users-db".password_resets (user_id, reset_token, expires_at) VALUES ($1, $2, $3) RETURNING id',
      [userId, resetToken, expiresAt]
    );
    return result.rows[0];
  }
};

module.exports = PasswordReset;
