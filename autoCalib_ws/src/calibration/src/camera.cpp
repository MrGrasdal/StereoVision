//
// Created by martin on 30.04.2021.
//

#include "camera.h"

Camera::Camera(double ts, bool right)
{
    CameraModel newModel(right);
    model = newModel;

    time = ts;

}

Camera::Camera(const Camera& sample)
{

    img = sample.img;

    allKpts = sample.allKpts;
    allDesc = sample.allDesc;
    currKpts = sample.currKpts;
    currDesc = sample.currDesc;
    epochKpts = sample.epochKpts;
    epochDesc = sample.epochDesc;

    model = sample.model;

    time = sample.time;

}

void Camera::initCamera(bool rght)
{
    CameraModel newModel(right);
    model = newModel;

    time = 0.0;
}


