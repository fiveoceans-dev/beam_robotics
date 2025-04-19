// routes/auth.js

const express = require('express');
const passport = require('passport');
const bcrypt = require('bcryptjs');
const { v4: uuidv4 } = require('uuid');
const User = require('../models/user');
const router = express.Router();

// Register route
router.post('/register', async (req, res, next) => {
  try {
    const { email, password } = req.body;
    const id = uuidv4();

    let user;
    try {
      user = await User.findByEmail(email);
    } catch (err) {
      // User not found, proceed with registration
    }

    if (user) {
      const isMatch = await bcrypt.compare(password, user.password_hash);
      if (!isMatch) {
        return res.redirect('/members?error=Incorrect password');
      }
      req.login(user, (err) => {
        if (err) return next(err);
        res.redirect('/account');
      });
    } else {
      user = await User.create({ email, id, password });
      req.login(user, (err) => {
        if (err) return next(err);
        res.redirect('/account');
      });
    }
  } catch (err) {
    console.error(err);
    res.redirect('/members?error=Registration failed');
  }
});

// Login route
router.post('/login', passport.authenticate('local', {
  successRedirect: '/account',
  failureRedirect: '/members?error=Invalid credentials',
}));

// Logout route
router.get('/logout', (req, res, next) => {
  req.logout((err) => {
    if (err) {
      return next(err);
    }
    res.redirect('/'); // Redirect to the home page or login page after logout
  });
});

module.exports = router;