#ifndef CAMERA_H
#define CAMERA_H

#include <QWidget>
#include <QQueue>

class Camera
{
public:
    Camera(QWidget *parent, QWidget * camera_1, QWidget * camera_2, QWidget * camera_3);
    void toggleMode();
    void toggleMain();

private:
    QWidget *parent;
    QWidget *camera_1;
    QWidget *camera_2;
    QWidget *camera_3;
    QQueue<QWidget *> cameras;

    enum CAMERA_LAYOUTS{main_only, main_corner, main_side};
    QQueue<CAMERA_LAYOUTS> camera_layouts;

    void setCamAsMain(QWidget * camera);
    void setCamAsCornerUpRight(QWidget * camera, int size_x, int size_y);
    void setCamAsCornerDownRight(QWidget * camera, int size_x, int size_y);
    void setCamAsCornerUpLeft(QWidget * camera, int size_x, int size_y);
    void setCamAsCornerDownLeft(QWidget * camera, int size_x, int size_y);

    void setCamAsSideLeft(QWidget * camera);
    void setCamAsSideRightUp(QWidget * camera);
    void setCamAsSideRightDown(QWidget * camera);

    void onCameraChange();

    float camera_corner_size_fraction = 0.25;
    float camera_side_size_fraction = 2/3;
    };

#endif // CAMERA_H
