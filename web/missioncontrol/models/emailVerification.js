const { Pool } = require('pg');

// Set up the connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false },
});

// EmailVerification model
const EmailVerification = {
  // Find email verification by userId and verificationCode
  findByUserIdAndCode: async (userId, verificationCode) => {
    const res = await pool.query(
      'SELECT * FROM "acorn-users-db".email_verifications WHERE user_id = $1 AND verification_code = $2',
      [userId, verificationCode]
    );
    return res.rows[0];
  },

  // Create a new email verification record
  create: async (userId, verificationCode) => {
    const expiresAt = new Date(Date.now() + 3600000); // 1 hour expiration time
    const result = await pool.query(
      'INSERT INTO "acorn-users-db".email_verifications (user_id, verification_code, expires_at) VALUES ($1, $2, $3) RETURNING id',
      [userId, verificationCode, expiresAt]
    );
    return result.rows[0];
  }
};

module.exports = EmailVerification;
