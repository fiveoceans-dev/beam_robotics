// ws.js

const WebSocket = require('ws');

/**
 * @param {http.Server} server  – the same server returned by http.createServer(...)
 */
function attachWebSocket(server) {
  // optional: isolate WS on its own path
  const wss = new WebSocket.Server({ server, path: '/ws' });

  wss.on('connection', (ws) => {
    console.log('WS client connected');

    ws.on('message', (message) => {
      // your existing CONTROL| and binary‐broadcast logic here
      handleMessage(wss, ws, message);
    });

    ws.on('close', () => console.log('WS client disconnected'));
  });
}

function handleMessage(wss, ws, message) {
  if (message instanceof Buffer) {
    console.log(`Received binary message (${message.byteLength} bytes)`);
    broadcast(wss, ws, message);
  } else {
    const text = message.toString();
    console.log(`Received text message: ${text}`);
    if (text.startsWith('CONTROL|')) {
      // parse + broadcast control message
      const commands = text.replace('CONTROL|','').split('+');
      let force = 0, turn = 0;
      commands.forEach(cmd => {
        if (cmd==='FORWARD')  force = 1;
        if (cmd==='BACKWARD') force = -1;
        if (cmd==='LEFT')     turn  = -1;
        if (cmd==='RIGHT')    turn  = 1;
      });
      broadcast(wss, ws, `${force},${turn}`);
    }
  }
}

function broadcast(wss, sender, data) {
  wss.clients.forEach(client => {
    if (client!==sender && client.readyState===WebSocket.OPEN) {
      client.send(data);
    }
  });
}

module.exports = { attachWebSocket };
