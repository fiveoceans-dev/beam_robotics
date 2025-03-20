export default function handler(req, res) {
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
  }
  