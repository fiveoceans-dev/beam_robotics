import asyncio
import websockets
import torch
import torchvision.transforms as transforms
import numpy as np
import cv2
from PIL import Image

# Load model
MODEL_PATH = "./model.pth"
device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")

class SegmentationModel(torch.nn.Module):
    def __init__(self):
        super(SegmentationModel, self).__init__()
        self.model = torch.hub.load("pytorch/vision:v0.10.0", "fcn_resnet50", pretrained=False)
        self.model.classifier[4] = torch.nn.Conv2d(512, 2, kernel_size=1)

    def forward(self, x):
        return self.model(x)['out']

# Load model
model = SegmentationModel().to(device)
model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
model.eval()

# Transform for input images
transform = transforms.Compose([
    transforms.Resize((640, 640)),
    transforms.ToTensor()
])

# Store connected clients (for forwarding processed frames)
connected_clients = set()

async def handle_incoming_frames(websocket, path):
    """Receives video frames from Unity, processes them, and forwards the output."""
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            # Convert received bytes to image
            img_array = np.frombuffer(message, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if img is None:
                print("Received invalid frame")
                continue

            # Convert to PIL for processing
            image_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            image_tensor = transform(image_pil).unsqueeze(0).to(device)

            # Predict free movement areas
            with torch.no_grad():
                output = model(image_tensor)
                output = torch.argmax(output.squeeze(), dim=0).cpu().numpy()

            # Overlay free areas in green
            overlay = np.zeros_like(img, dtype=np.uint8)
            overlay[output == 1] = [0, 255, 0]  # Green overlay for movement area
            processed_frame = cv2.addWeighted(img, 1, overlay, 0.5, 0)

            # Encode back to bytes
            _, buffer = cv2.imencode('.png', processed_frame)

            # Send to all connected clients on port 3008
            await broadcast_processed_frame(buffer.tobytes())

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected_clients.remove(websocket)

async def broadcast_processed_frame(frame_data):
    for client in connected_clients.copy():
        try:
            await client.send(frame_data)
        except:
            connected_clients.remove(client)

async def start_servers():
    print("WebSocket Server: Listening port 3007 and forwarding to port 3008")
    
    # Server for receiving frames from Unity (port 3009)
    unity_server = websockets.serve(handle_incoming_frames, "0.0.0.0", 3007)
    
    # Server for sending processed frames to clients (port 3008)
    processed_video_server = websockets.serve(broadcast_processed_frame, "0.0.0.0", 3008)

    await asyncio.gather(unity_server, processed_video_server)

if __name__ == "__main__":
    asyncio.run(start_servers())
