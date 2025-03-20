from torchsummary import summary
import torch

from torchvision.models.segmentation import fcn_resnet50

# Define the model (use the same architecture used in training)
model = fcn_resnet50(pretrained=False)
model.classifier[4] = torch.nn.Conv2d(512, 2, kernel_size=1)  # Adjust for your use case
model.load_state_dict(torch.load("./model.pth", map_location="cpu"))

# Print summary
summary(model, input_size=(3, 640, 640))

