// routes/api.js
require('dotenv').config();
const express = require('express');
const router = express.Router();

const OpenAI = require('openai');

const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

router.post('/chat', async (req, res) => {
  try {
    const { message, conversation } = req.body;
    const messages = conversation || [];
    messages.push({ role: 'user', content: message });

    const response = await openai.chat.completions.create({
      model: 'gpt-3.5-turbo',
      messages,
      temperature: 0.7,
    });

    const answer = response.choices[0].message;
    res.json({ answer, conversation: [...messages, answer] });

  } catch (error) {
    console.error(error);
    res.status(500).json({ error: 'OpenAI API error' });
  }
});

module.exports = router;
