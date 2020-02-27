#include "axisconfig.h"

AxisConfigs::AxisConfigs(JoystickAxis axis):
    AxisConfigs(axis, false, false) {}

AxisConfigs::AxisConfigs(JoystickAxis axis, bool isReversed, bool isAbsolute):
    AxisConfigs(axis, isReversed, isAbsolute, true) {}

AxisConfigs::AxisConfigs(JoystickAxis axis, bool isReversed, bool isAbsolute, bool isEnabled):
    isReversed(isReversed), isAbsolute(isAbsolute), isEnabled(isEnabled), axisDirection(Positive), axis(axis) {}

float AxisConfigs::scaleAndRoundValue() {
    if (!isEnabled)
        return 0;

    float scaledValue = isReversed? -value : value;

    if (isAbsolute) {
        scaledValue = (scaledValue + 1)/2;
        if (axisDirection == Negative)
            scaledValue = -scaledValue;
    }

    if (fabsf(scaledValue) < JOYSTICK_DEAD_MARGIN)
        return 0;
    else if (scaledValue > (1 - JOYSTICK_DEAD_MARGIN))
        return 1;
    else if (scaledValue < -(1 - JOYSTICK_DEAD_MARGIN))
        return -1;
    else
        return scaledValue;
}

void AxisConfigs::setValue(float value) {
    this->value = value;
}

void AxisConfigs::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void AxisConfigs::setDirection(AxisDirection direction) {
    axisDirection = direction;
}

void AxisConfigs::setDirectionAndEnable(AxisDirection direction) {
    setDirection(direction);
    setEnabled(true);
}

JoystickAxis AxisConfigs::getAxis() {
    return axis;
}
