require("dotenv").config();
const express = require("express");
const path = require("path");
const http = require("http");
const WebSocket = require("ws");

const app = express();
const PORT = process.env.PORT || 3009;
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

app.use(express.static(path.join(__dirname, "public")));

app.get("/api/config", (req, res) => {
  res.json({ googleMapsApiKey: process.env.GOOGLE_MAPS_API_KEY });
});

app.get("/api/weather-layer", (req, res) => {
  const { type } = req.query;
  if (!type) {
    return res.status(400).json({ error: "Missing weather layer type" });
  }

  const openWeatherApiKey = process.env.OPENWEATHER_API_KEY;
  if (!openWeatherApiKey) {
    return res.status(500).json({ error: "Weather API key not configured" });
  }

  const weatherLayerUrl = `https://tile.openweathermap.org/map/${type}/{z}/{x}/{y}.png?appid=${openWeatherApiKey}`;
  res.json({ url: weatherLayerUrl });
});

// WebSocket connection
wss.on("connection", (ws) => {
    console.log("Client connected");
  
    ws.on("message", (message) => {
      if (message instanceof Buffer) {
        // Forward image data to other clients
        wss.clients.forEach((client) => {
          if (client !== ws && client.readyState === WebSocket.OPEN) {
            client.send(message);
          }
        });
      } else {
        // Handle control commands
        let msgString = message.toString();
        if (msgString.startsWith("CONTROL|")) {
          let commands = msgString.replace("CONTROL|", "").split("+");
          console.log("Control commands received:", commands);
  
          let force = 0;
          let turn = 0;
  
          commands.forEach((command) => {
            switch (command) {
              case "FORWARD":
                force = 1;
                break;
              case "BACKWARD":
                force = -1;
                break;
              case "LEFT":
                turn = -1;
                break;
              case "RIGHT":
                turn = 1;
                break;
              default:
                console.log("Unknown command:", command);
            }
          });
  
          // Send force and turn values in the format "force,turn"
          let controlMessage = `${force},${turn}`;
          wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(controlMessage);
            }
          });
        } else {
          console.log("Received unknown data, ignoring...");
        }
      }
    });
  
    ws.on("close", () => console.log("Client disconnected"));
  });

// Start HTTP + WebSocket server
server.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});
