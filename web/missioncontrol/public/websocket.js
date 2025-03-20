// websocket.js

let socket;
const WS_URL = "ws://localhost:3000";
let activeKeys = new Set(); // Track pressed keys
let sendInterval = null; // Interval for sending commands
let imageCache = {}; // Cache last received images to avoid excessive reloading

function connectWebSocket() {
    socket = new WebSocket(WS_URL);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => {
        console.log("Connected to WebSocket server");
    };

    socket.onmessage = (event) => {
        if (event.data instanceof ArrayBuffer) {
            handleImageFrame(event.data);
        } else {
            handleControlMessage(event.data);
        }
    };

    socket.onclose = () => {
        console.warn("WebSocket disconnected. Attempting to reconnect...");
        setTimeout(connectWebSocket, 2000);
    };

    socket.onerror = (error) => {
        console.error("WebSocket error:", error);
        setTimeout(connectWebSocket, 2000);
    };
}

// Handle control messages (debugging)
function handleControlMessage(message) {
    console.log("Control message received:", message);
}

// Handle incoming video frames
function handleImageFrame(data) {
    let dataArray = new Uint8Array(data);
    let separatorIndex = dataArray.indexOf(124); // ASCII '|' separator
    if (separatorIndex === -1) {
        console.warn("Invalid image data received (no separator)");
        return;
    }

    let cameraName = new TextDecoder().decode(dataArray.slice(0, separatorIndex));
    let imageData = dataArray.slice(separatorIndex + 1);

    let blob = new Blob([imageData], { type: "image/jpeg" });
    let url = URL.createObjectURL(blob);

    // Prevent unnecessary reloads if the same image is received
    if (imageCache[cameraName] === url) return;
    imageCache[cameraName] = url;


    // Find existing placeholders inside the .stream-area div
    let imgContainers = document.querySelectorAll(".general-area img");

    for (let img of imgContainers) {
        if (img.getAttribute("data-camera") === cameraName) {
            img.src = url;
            return;
        }
    }

    console.warn(`No placeholder found for ${cameraName}, skipping frame.`);
}


// Establish WebSocket Connection
connectWebSocket();

// Continuously send active key commands
function startSendingCommands() {
    if (!sendInterval) {
        sendInterval = setInterval(() => {
            if (activeKeys.size > 0 && socket.readyState === WebSocket.OPEN) {
                let command = `CONTROL|${Array.from(activeKeys).join("+")}`;
                socket.send(command);
                // console.log("Sent:", command);
            }
        }, 100); // Send every 100ms
    }
}

// ⏹Stop sending when no keys are pressed
function stopSendingCommands() {
    if (activeKeys.size === 0 && sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
}

// Handle Key Press
document.addEventListener("keydown", (event) => {
    const keyMap = { "w": "FORWARD", "s": "BACKWARD", "a": "LEFT", "d": "RIGHT" };
    if (keyMap[event.key]) {
        activeKeys.add(keyMap[event.key]);
        startSendingCommands();
    }
});

// Handle Key Release
document.addEventListener("keyup", (event) => {
    const keyMap = { "w": "FORWARD", "s": "BACKWARD", "a": "LEFT", "d": "RIGHT" };
    if (keyMap[event.key]) {
        activeKeys.delete(keyMap[event.key]);
        stopSendingCommands();
    }
});
