#ifndef JOYSTICKHANDLER_H
#define JOYSTICKHANDLER_H

#include <QObject>
#include <map>
#include "joystick.h"

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

signals:
    void changeCameraMode();
    void axisChanged(const AxesValues &axesValues);

public slots:
    void onAxisChanged(JoystickAxis axis, float value);
    void onButtonAction(unsigned char btnNo, JoystickButtonAction action);

private:
    void initMap();

    void emitChangeCameraMode(JoystickButtonAction action);
    void emitRise(JoystickButtonAction action);
    void emitDown(JoystickButtonAction action);

    KeyMap m_map;
    float axesValues[4];
    Joystick* m_joystick;
    bool isRising;
    bool isDown;
};



#endif // JOYSTICKHANDLER_H
