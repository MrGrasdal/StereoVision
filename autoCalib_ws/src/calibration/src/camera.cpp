//
// Created by martin on 30.04.2021.
//

#include "camera.h"

Camera::Camera(double ts, bool right)
{
    CameraModel newModel(right);
    model = newModel;

    double time = ts;

}

Camera::Camera(const Camera& sample)
{

    img = sample.img;

    kpts = sample.kpts;
    desc = sample.desc;

    model = sample.model;

    time = sample.time;

}
