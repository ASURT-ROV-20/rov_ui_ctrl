#include "include/camera.h"

Camera::Camera(QWidget *parent, QWidget * camera_1, QWidget * camera_2, QWidget * camera_3) {
    this->parent = parent;
    this->camera_1 = camera_1;
    this->camera_2 = camera_2;
    this->camera_3 = camera_3;
    cameras.append(camera_1);
    cameras.append(camera_2);
    cameras.append(camera_3);

    camera_layouts.append(main_corner);
    camera_layouts.append(main_only);
    camera_layouts.append(main_side);
    onCameraChange();
}

void Camera::toggleMain() {
    QWidget * prev_main_cam = cameras[0];
    cameras.pop_front();
    cameras.push_back(prev_main_cam);
    onCameraChange();
}

void Camera::toggleMode() {
    CAMERA_LAYOUTS layout = camera_layouts[0];
    camera_layouts.pop_front();
    camera_layouts.push_back(layout);
    onCameraChange();
}

void Camera::onCameraChange() {
    cameras[0]->show();
    cameras[1]->show();
    cameras[2]->show();
    QSize size = parent->size();
    switch (camera_layouts[0]) {
        case main_only:
            setCamAsMain(cameras[0]);
            cameras[1]->hide();
            cameras[2]->hide();
        break;
        case main_side:
            setCamAsSideLeft(cameras[0]);
            setCamAsSideRightUp(cameras[1]);
            setCamAsSideRightDown(cameras[2]);
        break;
        case main_corner:
            setCamAsMain(cameras[0]);
            int corner_size_x = static_cast<int>(size.width() * CAMERA_CORNER_SIZE_FRACTION);
            int corner_size_y = static_cast<int>(size.height() * CAMERA_CORNER_SIZE_FRACTION);
            setCamAsCornerUpRight(cameras[1], corner_size_x, corner_size_y);
            setCamAsCornerUpLeft(cameras[2], corner_size_x, corner_size_y);
        break;
    }
}



void Camera::setCamAsSideLeft(QWidget * camera) {
    QSize size = parent->size();
    camera->setGeometry(0, 0,
                        static_cast<int>(size.width() * CAMERA_SIDE_SIZE_FRACTION),
                        size.height());
    camera->lower();
}

void Camera::setCamAsSideRightUp(QWidget * camera) {
    QSize size = parent->size();
    camera->setGeometry(static_cast<int>(size.width() * CAMERA_SIDE_SIZE_FRACTION),
                        0,
                        static_cast<int>(size.width() * (1 - CAMERA_SIDE_SIZE_FRACTION)),
                        size.height()/2);
    camera->lower();
}

void Camera::setCamAsSideRightDown(QWidget * camera) {
    QSize size = parent->size();
    camera->setGeometry(static_cast<int>(size.width() * CAMERA_SIDE_SIZE_FRACTION),
                        size.height()/2,
                        static_cast<int>(size.width() * (1 - CAMERA_SIDE_SIZE_FRACTION)),
                        size.height()/2);
    camera->lower();
}

void Camera::setCamAsMain(QWidget * camera) {
    QSize size = parent->size();
    camera->setGeometry(0, 0, size.width(), size.height());
    camera->lower(); //send widget to back
}

void Camera::setCamAsCornerUpRight(QWidget * camera, int size_x, int size_y) {
    QSize size = parent->size();
    int corner_anchor_up_x = static_cast<int>(size.width() * ( 1 - CAMERA_CORNER_SIZE_FRACTION));
    camera->setGeometry(corner_anchor_up_x, 0, size_x, size_y);
}

void Camera::setCamAsCornerUpLeft(QWidget * camera, int size_x, int size_y) {
    camera->setGeometry(0, 0, size_x, size_y);
}

void Camera::setCamAsCornerDownRight(QWidget * camera, int size_x, int size_y) {
    QSize size = parent->size();
    int corner_anchor_down_x = static_cast<int>(size.width() * ( 1 - CAMERA_CORNER_SIZE_FRACTION));
    int corner_anchor_down_y = static_cast<int>(size.height() * ( 1 - CAMERA_CORNER_SIZE_FRACTION));
    camera->setGeometry(corner_anchor_down_x, corner_anchor_down_y, size_x, size_y);
}

void Camera::setCamAsCornerDownLeft(QWidget * camera, int size_x, int size_y) {
    QSize size = parent->size();
    int corner_anchor_down_y = static_cast<int>(size.height() * ( 1 - CAMERA_CORNER_SIZE_FRACTION));
    camera->setGeometry(0, corner_anchor_down_y, size_x, size_y);
}
