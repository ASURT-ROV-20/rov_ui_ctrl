#ifndef JOYSTICKHANDLER_H
#define JOYSTICKHANDLER_H

#include <QDebug>
#include <QObject>
#include <map>
#include "joystick.h"
#include "axisconfig.h"

struct AxesValues {
    float x, y, z, r;
    AxesValues() {}
    AxesValues(float x, float y, float z, float r): x(x), y(y), z(z), r(r) {}
};
Q_DECLARE_METATYPE(AxesValues);

class JoystickHandler : public QObject
{
    Q_OBJECT

    typedef void(JoystickHandler::*KeyFunction)(JoystickButtonAction);
    typedef std::pair<unsigned char, KeyFunction> Pair;
    typedef std::map<unsigned char, KeyFunction> KeyMap;

public:
    explicit JoystickHandler(Joystick* joystick, QObject *parent = nullptr);
    ~JoystickHandler();

signals:
    void changeCameraMode();
    void changeMainCamera();
    void axisChanged(const AxesValues &axesValues);

public slots:
    void onAxisChanged(quint8 axis, float value);
    void onButtonAction(unsigned char btnNo, JoystickButtonAction action);

private:
    void initConfigs();

    void emitAxisChanged();
    void emitChangeCameraMode(JoystickButtonAction action);
    void emitChangeMainCamera(JoystickButtonAction action);
    void emitRise(JoystickButtonAction action);
    void emitDown(JoystickButtonAction action);

    KeyMap btn_map;
    std::map<quint8, AxisConfigs*> axes_map;
    AxisConfigs* axesConfigs[4];
    Joystick* m_joystick;
};



#endif // JOYSTICKHANDLER_H
