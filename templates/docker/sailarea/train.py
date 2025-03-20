import os
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import torchvision.models as models
import numpy as np
import cv2
from torch.utils.data import DataLoader, Dataset
from PIL import Image
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

# Paths
DATA_DIR = "./data"
MODEL_PATH = "./model.pth"
IMG_SIZE = 640

# Define Dataset
class BoatDataset(Dataset):
    def __init__(self, img_dir, transform=None):
        self.img_dir = img_dir
        self.transform = transform

        # Ensure directory exists
        if not os.path.exists(img_dir):
            print(f"Error: Dataset directory '{img_dir}' does not exist!")
            self.images = []
            self.masks = []
            return

        # List all images
        self.images = [f for f in os.listdir(img_dir) if f.endswith('.png')]
        self.masks = [f.replace(".png", "_mask.png") for f in self.images]

        # Filter only existing masks
        valid_pairs = [(img, mask) for img, mask in zip(self.images, self.masks) if os.path.exists(os.path.join(img_dir, mask))]

        if not valid_pairs:
            print(f"Error: No valid image-mask pairs found in '{img_dir}'")
            self.images, self.masks = [], []
        else:
            self.images, self.masks = zip(*valid_pairs)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = os.path.join(self.img_dir, self.images[idx])
        mask_path = os.path.join(self.img_dir, self.masks[idx])

        image = Image.open(img_path).convert("RGB")
        mask = Image.open(mask_path).convert("L")  # Load as grayscale

        if self.transform:
            image = self.transform(image)
            mask = self.transform(mask)

        return image, mask.squeeze(0)  # Remove channel dimension

# Define model
class SegmentationModel(nn.Module):
    def __init__(self):
        super(SegmentationModel, self).__init__()
        self.model = models.segmentation.fcn_resnet50(weights=models.segmentation.FCN_ResNet50_Weights.DEFAULT)
        self.model.classifier[4] = nn.Conv2d(512, 2, kernel_size=1)  # Adjust for 2 classes

    def forward(self, x):
        return self.model(x)['out']

# Training Function
def train():
    transform = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor()
    ])

    dataset = BoatDataset(DATA_DIR, transform)
    
    if len(dataset) == 0:
        print("Dataset is empty! Check if images and masks exist in the data folder.")
        return
    
    print(f"Loaded {len(dataset)} images and masks for training.")

    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)

    device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
    model = SegmentationModel().to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.CrossEntropyLoss()

    for epoch in range(10):
        total_loss = 0
        for images, masks in dataloader:
            images, masks = images.to(device), masks.long().to(device)  # Ensure mask is in correct format

            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, masks)
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        print(f"Epoch {epoch+1}, Loss: {total_loss / len(dataloader)}")

    torch.save(model.state_dict(), MODEL_PATH)
    print("Model saved!")

if __name__ == "__main__":
    train()