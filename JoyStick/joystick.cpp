#include <QDebug>
#include "joystick.h"

Joystick* Joystick::m_instance = nullptr;

float convertPercent(float v) {
    return -(v)/powf(2, 15);
}

Joystick::Joystick():
    m_controller(nullptr), m_isConnected(false), m_isRunning(true), m_thread(new QThread(this)) {

    connect(m_thread, &QThread::started, this, &Joystick::run);
    moveToThread(m_thread);
    m_thread->start();
}

Joystick::~Joystick() {
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
    if( SDL_Init(SDL_INIT_JOYSTICK) < 0 )
        return false;

    if( SDL_NumJoysticks() < 1 ) {
        return false;
    } else {
        m_controller = SDL_JoystickOpen( 0 );
        if (m_controller == nullptr)
            return false;
    }
    return true;
}

void Joystick::close() {
    m_isRunning = false;
    SDL_JoystickClose(m_controller);
    m_isConnected = false;
}

bool Joystick::isConnected() { return m_isConnected; }

void Joystick::run() {
    while (!init() && m_isRunning) {
        QThread::msleep(100);
    }
    emit connected();

    SDL_Event e;
    float prevX = 0, prevY = 0, prevZ = 0, prevR = 0;
    while(m_isRunning)
    {
        while (SDL_PollEvent(&e)) {
            //User requests quit
            if( e.type == SDL_QUIT )
            {
                close();
            } else if( e.type == SDL_JOYAXISMOTION ) {
                if ((fabs(SDL_JoystickGetAxis(m_controller, 0) - prevX) > JOYSTICK_DEAD_ZONE) ||
                        (fabs(SDL_JoystickGetAxis(m_controller, 1) - prevY) > JOYSTICK_DEAD_ZONE) ||
                        (fabs(SDL_JoystickGetAxis(m_controller, 2) - prevZ) > JOYSTICK_DEAD_ZONE) ||
                        (fabs(SDL_JoystickGetAxis(m_controller, 3) - prevR) > JOYSTICK_DEAD_ZONE)) {
                    prevX = SDL_JoystickGetAxis(m_controller, 0);
                    prevY = SDL_JoystickGetAxis(m_controller, 1);
                    prevZ = SDL_JoystickGetAxis(m_controller, 2);
                    prevR = SDL_JoystickGetAxis(m_controller, 3);
                    qDebug() << e.jaxis.value;
                    emit axisChanged(convertPercent(prevX), convertPercent(prevY), (convertPercent(prevZ) + 1 )/ 2, convertPercent(prevR));
                }
            } else if (e.type == SDL_JOYHATMOTION) {
                qDebug() << e.jhat.value;
            } else if ( e.type == SDL_JOYBUTTONUP ) {
                emit buttonUp(e.jbutton.button);
            } else if ( e.type == SDL_JOYBUTTONDOWN ) {
                emit buttonPressed(e.jbutton.button);
            }
        }
    }

    emit disconnected();
}
