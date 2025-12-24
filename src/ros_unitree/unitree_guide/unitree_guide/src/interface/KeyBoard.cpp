/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
    
    // ✅ Subscribe to /cmd_vel
    sub_cmd_ = nh_.subscribe("/cmd_vel", 10, &KeyBoard::cmdVelCallback, this);
}

void KeyBoard::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // ✅ Forward/backward
    //userValue.ly = std::max(std::min((float)msg->linear.x,  1.0f), -1.0f);
    // ✅ Lateral (left/right)
    //userValue.lx = std::max(std::min((float)msg->linear.y,  1.0f), -1.0f);
    // ✅ Pitch 
    //userValue.ry = std::max(std::min((float)msg->angular.y, 1.0f), -1.0f);
    // ✅ Yaw
    //userValue.rx = std::max(std::min((float)msg->angular.z, 1.0f), -1.0f);
    // ✅ Mode selection using linear.z
    int mode = static_cast<int>(std::round(msg->linear.z));
    switch (mode) {
        case 1:
            userCmd = UserCommand::L2_B; break;
        case 2:
            userCmd = UserCommand::L2_A; break;
        case 3:
            userCmd = UserCommand::L2_X; break;
        case 4:
            userCmd = UserCommand::START; break;
#ifdef COMPILE_WITH_MOVE_BASE
        case 5:
            userCmd = UserCommand::L2_Y; break;
#endif
        case 0:
            userCmd = UserCommand::L1_X; break;
        case 9:
            userCmd = UserCommand::L1_A; break;
        case 8:
            userCmd = UserCommand::L1_Y; break;
        default:
            userCmd = UserCommand::NONE; break;
    }
}


KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0); break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0); break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0); break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0); break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0); break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0); break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0); break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0); break;
    default: break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}
