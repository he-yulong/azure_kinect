from torchvision import models
from torch import nn
from torchvision.models.resnet import *
from global_config import *


class blenderNet(nn.Module):

    def __init__(self, gpu=USE_GPU):
        super(blenderNet, self).__init__()
        self.model_ft = resnet18(pretrained=False)
        # print(self.model_ft.avgpool.output_size)
        num_features = self.model_ft.fc.in_features
        self.model_ft.fc = nn.Linear(num_features, 52)
        torch.nn.init.kaiming_normal_(self.model_ft.fc.weight)
        self.sigmoid = nn.Sigmoid()
        if gpu:
            self.model_ft = self.model_ft.cuda()
        torch.nn.init.kaiming_normal_(self.model_ft.fc.weight)

    def forward(self, x):
        x = self.model_ft.conv1(x)
        x = self.model_ft.bn1(x)
        x = self.model_ft.relu(x)
        x = self.model_ft.maxpool(x)

        x = self.model_ft.layer1(x)
        x = self.model_ft.layer2(x)
        x = self.model_ft.layer3(x)
        x = self.model_ft.layer4(x)

        x = self.model_ft.avgpool(x)
        x = x.view(x.size(0), -1)
        x = self.model_ft.fc(x)
        x = self.sigmoid(x)
        return x

    def train(self, mode=True):
        self.model_ft.train()

    def eval(self):
        self.model_ft.eval()


def fine_tune_model():
    model_ft = models.resnet18(pretrained=True)
    num_features = model_ft.fc.in_features
    # fine tune we change original fc layer into classes num of our own
    #human
    model_ft.fc = nn.Linear(num_features, 52)
   #print model_ft
    torch.nn.init.kaiming_normal_(model_ft.fc.weight)


    if USE_GPU:
        model_ft = model_ft.cuda()
    return model_ft
