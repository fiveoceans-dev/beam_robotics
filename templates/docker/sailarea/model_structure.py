import torch

MODEL_PATH = "./model.pth"

# Load the model
model = torch.load(MODEL_PATH, map_location=torch.device("cpu"))
# print(model)

state_dict = torch.load(MODEL_PATH, map_location=torch.device("cpu"))

# Print available layer names
print("Model State Dict Keys:")
print(state_dict.keys())

# Print shape of some layers
for name, param in state_dict.items():
    print(f"{name}: {param.shape}")
