// api.js (Deploy this as a Vercel Serverless Function under /api/[...routes].js)

module.exports = async (req, res) => {
    const { GOOGLE_MAPS_API_KEY, OPENWEATHER_API_KEY } = process.env;
  
    if (req.url.startsWith('/api/googlemaps')) {
      if (!GOOGLE_MAPS_API_KEY) {
        return res.status(500).json({ error: "Google Maps API key not configured" });
      }
      return res.json({ googleMapsApiKey: GOOGLE_MAPS_API_KEY });
    }
  
    if (req.url.startsWith('/api/weather')) {
      const type = req.query.type;
      if (!type) {
        return res.status(400).json({ error: "Missing weather layer type" });
      }
  
      if (!OPENWEATHER_API_KEY) {
        return res.status(500).json({ error: "Weather API key not configured" });
      }
  
      const weatherLayerUrl = `https://tile.openweathermap.org/map/${type}/{z}/{x}/{y}.png?appid=${OPENWEATHER_API_KEY}`;
      return res.json({ url: weatherLayerUrl });
    }
  
    res.status(404).json({ error: "API route not found" });
  };
  