let map;
let currentWeatherLayer = null;

function initMap() {
    map = new google.maps.Map(document.getElementById('map'), {
        center: { lat: 43.7365, lng: 7.4284 },
        zoom: 19,
        mapTypeId: 'satellite',
        tilt: 0,
        heading: 0,
        disableDefaultUI: false,
        gestureHandling: 'greedy',
        zoomControl: true,
        rotateControl: false,
        streetViewControl: false,
    });
}

async function toggleWeatherLayer(type) {
    try {
        if (currentWeatherLayer) {
            map.overlayMapTypes.pop();
            currentWeatherLayer = null;
        }

        const response = await fetch(`/api/weather-layer?type=${type}`);
        const data = await response.json();

        if (data.error) {
            console.error("Weather Layer Error:", data.error);
            return;
        }

        currentWeatherLayer = new google.maps.ImageMapType({
            getTileUrl: function(coord, zoom) {
                return data.url
                    .replace("{z}", zoom)
                    .replace("{x}", coord.x)
                    .replace("{y}", coord.y);
            },
            tileSize: new google.maps.Size(256, 256),
            opacity: 0.6,
            name: type
        });

        map.overlayMapTypes.push(currentWeatherLayer);
        console.log("Weather layer added:", type);

    } catch (error) {
        console.error("Failed to load weather layer:", error);
    }
}

async function loadGoogleMaps() {
    try {
        const response = await fetch('/api/config');
        const config = await response.json();

        if (!config.googleMapsApiKey) {
            console.error("Google Maps API key not found.");
            return;
        }

        const script = document.createElement("script");
        script.src = `https://maps.googleapis.com/maps/api/js?key=${config.googleMapsApiKey}&callback=initMap&libraries=places`;
        script.async = true;
        script.defer = true;
        document.body.appendChild(script);
    } catch (error) {
        console.error("Failed to load API keys:", error);
    }
}

window.onload = loadGoogleMaps;
