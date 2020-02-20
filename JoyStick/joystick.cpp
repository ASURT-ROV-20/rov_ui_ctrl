#include <QDebug>
#include "joystick.h"

Joystick* Joystick::m_instance = nullptr;

Joystick::Joystick():
    m_controller(nullptr), m_isRunning(true), m_thread(new QThread(this)) {
    qRegisterMetaType<JoystickAxis>();
    qRegisterMetaType<JoystickButtonAction>();
    connect(m_thread, &QThread::started, this, &Joystick::run);
    moveToThread(m_thread);
    m_thread->start();
}

Joystick::~Joystick() {
    shutdown();
    if (m_thread->isRunning()) {
        m_thread->exit();
    }
    m_thread->wait(1000);
    delete m_thread;
    m_thread = nullptr;
}

Joystick* Joystick::getInstance() {
//    m_instance_lock.lock();
    if (m_instance == nullptr)
        m_instance = new Joystick();
//    m_instance_lock.unlock();
    return m_instance;
}

bool Joystick::init() {
    if( SDL_Init(SDL_INIT_JOYSTICK) < 0 ) {
        qWarning() << "Failed to initialize SDL Joystick. Error: " << SDL_GetError();
        return false;
    }
    return true;
}

bool Joystick::openJoystick(int deviceIndex) {
    if (SDL_NumJoysticks() < 1) {
        qWarning() << "No joysticks connected";
        return false;
    }

    m_controller = SDL_JoystickOpen(deviceIndex);
    if (m_controller == nullptr) {
        qWarning() << "Failed to open Joystick, Name: " << SDL_JoystickNameForIndex(0) << ". Error: " << SDL_GetError();
        return false;
    }

    emit connected();
    return true;
}

void Joystick::close() {
    SDL_JoystickClose(m_controller);
    m_controller = nullptr;
    emit disconnected();
}

void Joystick::shutdown() {
    m_isRunning = false;
    close();
}

bool Joystick::isConnected() { return (m_controller != nullptr); }

float Joystick::scaleAxisValue(JoystickAxis axis, float value) {
    if (axis == AxisZ) {
        value = (-value + JOYSTICK_SCALE)/2;
        if (value < JOYSTICK_MIN_ZAXIS_VALUE)
            return 0;
        if (value > JOYSTICK_MAX_ZAXIS_VALUE)
            return 1;
        return value/JOYSTICK_SCALE;
    } else {
        if (abs(value) < JOYSTICK_DEAD_ZONE)
            return 0;
        if (value < JOYSTICK_MIN_AXIS_VALUE)
            return 1;
        if (value > JOYSTICK_MAX_AXIS_VALUE)
            return -1;
        return -value/JOYSTICK_SCALE;
    }
}

void Joystick::run() {
    while (!init() && m_isRunning) {
        QThread::msleep(100);
    }

    SDL_Event e;
    float prevValues[] = {0, 0, 0, 0};
    while(m_isRunning)
    {
        while (SDL_PollEvent(&e)) {
            if (m_controller == nullptr && e.type != SDL_JOYDEVICEADDED)
                continue;

            //User requests quit
            if( e.type == SDL_QUIT )
            {
                shutdown();
            } else if( e.type == SDL_JOYAXISMOTION ) {
                if (fabs(prevValues[e.jaxis.axis] - e.jaxis.value) > JOYSTICK_CHANGE_INTEVAL) {
                    prevValues[e.jaxis.axis] = e.jaxis.value;
                    float scaledValue = scaleAxisValue((JoystickAxis)(e.jaxis.axis), e.jaxis.value);
                    emit axisChanged((JoystickAxis)(e.jaxis.axis), scaledValue);
                }
            } else if ( e.type == SDL_JOYBUTTONUP ) {
                emit buttonAction(e.jbutton.button, Up);
            } else if ( e.type == SDL_JOYBUTTONDOWN ) {
                emit buttonAction(e.jbutton.button, Down);
            } else if ( e.type == SDL_JOYDEVICEREMOVED && e.jdevice.which == SDL_JoystickInstanceID(m_controller) ) {
                close();
                openJoystick(0);
            } else if ( e.type == SDL_JOYDEVICEADDED && m_controller == nullptr ) {
                openJoystick(e.jdevice.which);
            }
        }
    }

    emit disconnected();
}

QString Joystick::getName() {
    return SDL_JoystickName(m_controller);
}

QString Joystick::getGuid() {
    char guid[16];
    SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(m_controller), guid, sizeof (guid));
    return guid;
}
