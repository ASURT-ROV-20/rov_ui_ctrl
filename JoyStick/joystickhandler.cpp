#include "joystickhandler.h"

JoystickHandler::JoystickHandler(Joystick* joystick, QObject *parent) :
    QObject(parent), axesValues{0,0,0,0}, m_joystick(joystick), isRising(false), isDown(false)
{
    qRegisterMetaType<AxesValues>();
    initMap();
    connect(joystick, &Joystick::buttonAction, this, &JoystickHandler::onButtonAction);
    connect(joystick, &Joystick::axisChanged, this, &JoystickHandler::onAxisChanged);
}

void JoystickHandler::initMap() {
    m_map.insert(Pair(0, &JoystickHandler::emitChangeCameraMode));
    m_map.insert(Pair(2, &JoystickHandler::emitRise));
    m_map.insert(Pair(3, &JoystickHandler::emitDown));

}

void JoystickHandler::onButtonAction(unsigned char btnNo, JoystickButtonAction action) {
    auto p = m_map.find(btnNo);
    if (p != m_map.end()) {
        (this->*(p->second))(action);
    }
}

void JoystickHandler::emitChangeCameraMode(JoystickButtonAction action) {
    if (action == Down) {
        emit changeCameraMode();
    }
}

void JoystickHandler::emitRise(JoystickButtonAction action) {
    isRising = (action == Down);
    float zValue = (action == Down? axesValues[2] : 0);
    emit axisChanged(AxesValues(axesValues[0], axesValues[1], zValue, axesValues[3]));
}

void JoystickHandler::emitDown(JoystickButtonAction action) {
    isDown = (action == Down);
    float zValue = (action == Down? -axesValues[2] : 0);
    emit axisChanged(AxesValues(axesValues[0], axesValues[1], zValue, axesValues[3]));
}

void JoystickHandler::onAxisChanged(JoystickAxis axis, float value) {
    axesValues[axis] = value;
    if (axis == AxisZ && !isRising && !isDown)
        return;

    float zValue = (isRising? axesValues[2] : (isDown? -axesValues[2] : 0));
    emit axisChanged(AxesValues(AxesValues(axesValues[0], axesValues[1], zValue, axesValues[3])));
}
