// websocket.js

let socket = null;
let activeKeys = new Set();
let sendInterval = null;
let imageCache = {};
let isConnected = false;

// Start or stop WebSocket connection
document.getElementById("toggle-btn").addEventListener("click", () => {
    const url = document.getElementById("websocket").value.trim() || "ws://localhost:3000";

    if (!isConnected) {
        connectWebSocket(url);
    } else {
        closeWebSocket();
    }
});

// Connect to WebSocket server
function connectWebSocket(url) {
    socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => {
        isConnected = true;
        updateStatusDot(true);
        document.getElementById("toggle-btn").textContent = "Stop";
        console.log("Connected to WebSocket server:", url);
    };

    socket.onmessage = (event) => {
        if (event.data instanceof ArrayBuffer) {
            handleImageFrame(event.data);
        } else {
            handleControlMessage(event.data);
        }
    };

    socket.onclose = () => {
        isConnected = false;
        updateStatusDot(false);
        document.getElementById("toggle-btn").textContent = "Start";
        console.warn("WebSocket disconnected.");
    };

    socket.onerror = (error) => {
        console.error("WebSocket error:", error);
    };
}

// Close WebSocket connection
function closeWebSocket() {
    if (socket) {
        socket.close();
        socket = null;
        isConnected = false;
        updateStatusDot(false);
        document.getElementById("toggle-btn").textContent = "Start";
        console.log("WebSocket closed manually.");
    }
}

// Update dot indicator based on WebSocket status
function updateStatusDot(active) {
    const statusDot = document.getElementById("ws-status");
    statusDot.className = active ? "dot-active" : "dot-idle";
}

// Handle incoming video frames
function handleImageFrame(data) {
    const dataArray = new Uint8Array(data);
    const separatorIndex = dataArray.indexOf(124);
    if (separatorIndex === -1) {
        console.warn("Invalid image data received (no separator).");
        return;
    }

    const cameraName = new TextDecoder().decode(dataArray.slice(0, separatorIndex));
    const imageData = dataArray.slice(separatorIndex + 1);
    const blob = new Blob([imageData], { type: "image/jpeg" });
    const url = URL.createObjectURL(blob);

    if (imageCache[cameraName] === url) return;
    imageCache[cameraName] = url;

    const imgContainers = document.querySelectorAll(".general-area img");
    imgContainers.forEach((img) => {
        if (img.getAttribute("data-camera") === cameraName) {
            img.src = url;
        }
    });
}

// Handle incoming text control messages
function handleControlMessage(message) {
    console.log("Control message received:", message);
}

// Send commands at intervals when keys are pressed
function startSendingCommands() {
    if (!sendInterval) {
        sendInterval = setInterval(() => {
            if (activeKeys.size > 0 && socket && socket.readyState === WebSocket.OPEN) {
                const command = `CONTROL|${Array.from(activeKeys).join("+")}`;
                socket.send(command);
            }
        }, 100);
    }
}

// Stop sending when no keys pressed
function stopSendingCommands() {
    if (activeKeys.size === 0 && sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
}

// Handle keyboard events
document.addEventListener("keydown", (event) => {
    const keyMap = { "w": "FORWARD", "s": "BACKWARD", "a": "LEFT", "d": "RIGHT" };
    if (keyMap[event.key]) {
        activeKeys.add(keyMap[event.key]);
        startSendingCommands();
    }
});

document.addEventListener("keyup", (event) => {
    const keyMap = { "w": "FORWARD", "s": "BACKWARD", "a": "LEFT", "d": "RIGHT" };
    if (keyMap[event.key]) {
        activeKeys.delete(keyMap[event.key]);
        stopSendingCommands();
    }
});

// Initialize UI
updateStatusDot(false);
document.getElementById("toggle-btn").textContent = "Start";