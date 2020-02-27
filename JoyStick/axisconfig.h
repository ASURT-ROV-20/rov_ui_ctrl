#ifndef AXISCONFIG_H
#define AXISCONFIG_H

#include <math.h>

#define JOYSTICK_DEAD_MARGIN .08f

enum JoystickAxis {AxisX = 0, AxisY = 1, AxisZ = 2, AxisR = 3};
enum AxisDirection {Positive, Negative};

class AxisConfigs
{
public:
    AxisConfigs(JoystickAxis axis);
    AxisConfigs(JoystickAxis axis, bool isReversed, bool isAbsolute);
    AxisConfigs(JoystickAxis axis, bool isReversed, bool isAbsolute, bool isEnabled);

    float scaleAndRoundValue();
    JoystickAxis getAxis();
    void setDirection(AxisDirection direction);
    void setDirectionAndEnable(AxisDirection direction);
    void setEnabled(bool enabled);
    void setValue(float value);

private:
    bool isReversed, isAbsolute, isEnabled;
    float value;
    AxisDirection axisDirection;
    JoystickAxis axis;
};

#endif // AXISCONFIG_H
