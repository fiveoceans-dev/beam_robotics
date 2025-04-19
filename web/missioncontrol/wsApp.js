const express = require('express');
const path    = require('path');
const api     = require('./routes/api');        // your existing API routes

const app = express();

// if you have a special template for the WebSocket UI:
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));

// static assets (JS client, CSS, etc.)
app.use(express.static(path.join(__dirname, 'public')));

// mount your API under /api if needed
app.use('/api', api);

// render the WS client page
app.get('/', (req, res) => {
  res.render('websocket');  // views/websocket.ejs : your page that opens the WS connection
});

module.exports = app;
