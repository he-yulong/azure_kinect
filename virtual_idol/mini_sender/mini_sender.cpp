// mini_sender.cpp : This file contains the 'main' function. Program execution begins and ends there.
// ц╩сп OpenCV

#include <iostream>
#include "SingleKinect.h"

int main()
{
    ws_tech::SingleKinect kinect = ws_tech::SingleKinect(0, "127.0.0.1", 8998, "unity");
    kinect.Open();
    kinect.Running(30 * 60);
    kinect.Close();
}
