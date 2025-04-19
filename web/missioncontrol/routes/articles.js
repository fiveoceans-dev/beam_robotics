// routes/articles.js

const express = require('express');
const router = express.Router();

// Route for the Community article
router.get('/community', (req, res) => {
    res.render('articles/article0', { title: 'Acorn SH - Community Article', description: 'Building a community in decentralized computing.' });
});


module.exports = router;
