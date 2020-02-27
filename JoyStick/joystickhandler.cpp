#include "joystickhandler.h"

JoystickHandler::JoystickHandler(Joystick* joystick, QObject *parent) :
    QObject(parent), m_joystick(joystick)
{
    qRegisterMetaType<AxesValues>();
    initConfigs();
    connect(joystick, &Joystick::buttonAction, this, &JoystickHandler::onButtonAction);
    connect(joystick, &Joystick::axisChanged, this, &JoystickHandler::onAxisChanged);
}

void JoystickHandler::initConfigs() {
    btn_map.insert(Pair(0, &JoystickHandler::emitChangeCameraMode));
    btn_map.insert(Pair(1, &JoystickHandler::emitChangeMainCamera));
    btn_map.insert(Pair(4, &JoystickHandler::emitRise));
    btn_map.insert(Pair(2, &JoystickHandler::emitDown));

    axesConfigs[AxisX] = new AxisConfigs(AxisX, false, false);
    axesConfigs[AxisY] = new AxisConfigs(AxisY, true, false);
    axesConfigs[AxisZ] = new AxisConfigs(AxisZ, true, true, false);
    axesConfigs[AxisR] = new AxisConfigs(AxisR, false, false);

    axes_map.insert(std::make_pair(0, axesConfigs[AxisX]));
    axes_map.insert(std::make_pair(1, axesConfigs[AxisY]));
    axes_map.insert(std::make_pair(3, axesConfigs[AxisZ]));
    axes_map.insert(std::make_pair(2, axesConfigs[AxisR]));
}

void JoystickHandler::onButtonAction(unsigned char btnNo, JoystickButtonAction action) {
    auto p = btn_map.find(btnNo);
    if (p != btn_map.end()) {
        (this->*(p->second))(action);
    }
}

void JoystickHandler::emitChangeCameraMode(JoystickButtonAction action) {
    if (action == Down) {
        emit changeCameraMode();
    }
}

void JoystickHandler::emitChangeMainCamera(JoystickButtonAction action) {
    if (action == Down) {
        emit changeMainCamera();
    }
}

void JoystickHandler::emitRise(JoystickButtonAction action) {
    if (action == Down)
        axesConfigs[AxisZ]->setDirectionAndEnable(Positive);
    else
        axesConfigs[AxisZ]->setEnabled(false);
    emitAxisChanged();
}

void JoystickHandler::emitAxisChanged() {
    AxesValues axesValues = AxesValues(axesConfigs[AxisX]->scaleAndRoundValue(),
                                       axesConfigs[AxisY]->scaleAndRoundValue(),
                                       axesConfigs[AxisZ]->scaleAndRoundValue(),
                                       axesConfigs[AxisR]->scaleAndRoundValue());
    emit axisChanged(axesValues);
}

void JoystickHandler::emitDown(JoystickButtonAction action) {
    if (action == Down)
        axesConfigs[AxisZ]->setDirectionAndEnable(Negative);
    else
        axesConfigs[AxisZ]->setEnabled(false);
    emitAxisChanged();
}

void JoystickHandler::onAxisChanged(quint8 axis, float value) {
    auto axisConfigsIt = axes_map.find(axis);
    if (axisConfigsIt == axes_map.end()) {
        qWarning() << "Can't find configs for axis " << axis;
        return;
    }

    AxisConfigs* axisConfig = axisConfigsIt->second;
    axisConfig->setValue(value);
    emitAxisChanged();
}

JoystickHandler::~JoystickHandler() {
    delete axes_map[0];
    delete axes_map[1];
    delete axes_map[2];
    delete axes_map[3];
}
