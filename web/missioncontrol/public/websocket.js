// websocket.js

let socket = null;
let activeKeys = new Set();
let sendInterval = null;
let imageCache = {};
let isConnected = false;

// Start or stop WebSocket connection
document.addEventListener("DOMContentLoaded", () => {
    const toggleBtn = document.getElementById("toggle-btn");
    const websocketInput = document.getElementById("websocket");

    let socket = null;
    let activeKeys = new Set();
    let sendInterval = null;
    let imageCache = {};
    let isConnected = false;

    toggleBtn.addEventListener("click", () => {
        const url = websocketInput.value.trim() || "ws://localhost:3000";

        if (!isConnected) {
            connectWebSocket(url);
        } else {
            closeWebSocket();
        }
    });

    function connectWebSocket(url) {
        socket = new WebSocket(url);
        socket.binaryType = "arraybuffer";

        socket.onopen = () => {
            isConnected = true;
            updateStatusDot(true);
            toggleBtn.textContent = "Stop";
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
            toggleBtn.textContent = "Start";
            console.warn("WebSocket disconnected.");
        };

        socket.onerror = (error) => {
            console.error("WebSocket error:", error);
        };
    }

    function closeWebSocket() {
        if (socket) {
            socket.close();
            socket = null;
            isConnected = false;
            updateStatusDot(false);
            toggleBtn.textContent = "Start";
            console.log("WebSocket closed manually.");
        }
    }

    function updateStatusDot(active) {
        const statusDot = document.getElementById("ws-status");
        statusDot.className = active ? "dot-active" : "dot-idle";
    }

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

    function handleControlMessage(message) {
        console.log("Control message received:", message);
    }

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

    function stopSendingCommands() {
        if (activeKeys.size === 0 && sendInterval) {
            clearInterval(sendInterval);
            sendInterval = null;
        }
    }

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

    updateStatusDot(false);
    toggleBtn.textContent = "Start";
});
