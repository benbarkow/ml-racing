import torch.nn as nn
import torch
import torch.nn.functional as F

class CNN_4LayersDrop(nn.Module):
    def __init__(self):
        super(CNN_4LayersDrop, self).__init__()

        # Convolutional layers
        self.conv1 = nn.Conv2d(9, 32, kernel_size=3, stride=1, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        self.dropout1 = nn.Dropout2d(p=0.2)

        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.dropout2 = nn.Dropout2d(p=0.2)

        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(128)
        self.dropout3 = nn.Dropout2d(p=0.2)
        
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)
        self.bn4 = nn.BatchNorm2d(256)
        self.dropout4 = nn.Dropout2d(p=0.2)

        # Flatten output from last layer; you'll need to adjust the size based on the input size and pooling operations
        self.fc1 = nn.Linear(17920, 512)  
        self.fc2 = nn.Linear(512, 128)
        self.fc3 = nn.Linear(128, 8)  # Output size based on previous example

    def forward(self, x):
        x = self.dropout1(F.relu(self.bn1(self.conv1(x))))
        x = F.max_pool2d(x, 2) 
        
        x = self.dropout2(F.relu(self.bn2(self.conv2(x))))
        x = F.max_pool2d(x, 2) 
        
        x = self.dropout3(F.relu(self.bn3(self.conv3(x))))
        x = F.max_pool2d(x, 2) 

        x = self.dropout4(F.relu(self.bn4(self.conv4(x))))

        x = x.view(x.size(0), -1)  # Flatten
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = torch.sigmoid(self.fc3(x))
        
        return x