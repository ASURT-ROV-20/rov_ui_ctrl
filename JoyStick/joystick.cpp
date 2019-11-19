#include <QDebug>
#include "joystick.h"

//Joystick::Joystick(QObject *parent) : QObject(parent)
//{
//    SDL_Init(SDL_INIT_JOYSTICK);
//    btn_pressed = false;
//    btns = SDL_JoystickNumButtons(myJoystick);
//    myJoystick = SDL_JoystickOpen(0);
//    if(myJoystick == NULL)
//    {
//        qDebug() << "No joystick connected";
//        exit(1);
//    }
//    qDebug() << SDL_JoystickNumAxes(myJoystick);
//    qDebug() << SDL_JoystickNumButtons(myJoystick);
//    //Initiate the publisher and start it, make it of type int
//    prevx = 0; prevy = 0; prevz = 0;
//}

//bool Joystick::pressed() { return btn_pressed; }

//int Joystick::getX() { return x; }

//int Joystick::getY() { return y; }

//int Joystick::getZ() { return z; }

//int Joystick::getBtn_value() { return btn_value; }

//void Joystick::joystick_update()
//{
//    if(abs(SDL_JoystickGetAxis(myJoystick,0) - prevx) > 2500)
//    {
//        x = SDL_JoystickGetAxis(myJoystick,0);
//    }
//    if(abs(SDL_JoystickGetAxis(myJoystick,1) - prevy) > 2500)
//    {
//        y = SDL_JoystickGetAxis(myJoystick,1);
//    }
//    if(abs(SDL_JoystickGetAxis(myJoystick,2) - prevz) > 2500)
//    {
//        z = SDL_JoystickGetAxis(myJoystick,2);
//    }
//    for (int i = 0;i<btns;i++)
//    {
//        if(SDL_JoystickGetButton(myJoystick,i) == 1)
//        {
//            btn_value = i;
//            btn_pressed = true;
//            break;
//        }
//        else
//        {
//          btn_pressed = false;
//        }
//    }
//}


Joystick* Joystick::m_instance = nullptr;

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
    float prevX = 0, prevY = 0, prevZ = 0;
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
                        (fabs(SDL_JoystickGetAxis(m_controller, 2) - prevZ) > JOYSTICK_DEAD_ZONE)) {
                    prevX = SDL_JoystickGetAxis(m_controller, 0);
                    prevY = SDL_JoystickGetAxis(m_controller, 1);
                    prevZ = SDL_JoystickGetAxis(m_controller, 2);
                    emit axisChanged(prevX, prevY, prevZ);
                }
            } else if ( e.type == SDL_JOYBUTTONUP ) {
                emit buttonUp(e.jbutton.button);
            } else if ( e.type == SDL_JOYBUTTONDOWN ) {
                emit buttonPressed(e.jbutton.button);
            }
        }
    }

    emit disconnected();
}
