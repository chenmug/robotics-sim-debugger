#include "robots/Robot.hpp"


/****************** SET ID ******************/

void Robot::setID(size_t id) 
{ 
    id_ = id; 
}


/****************** GET ID ******************/

size_t Robot::getID() const 
{ 
    return id_; 
}


/***************** ADD SENSOR ****************/

void Robot::addSensor(std::shared_ptr<Sensor> sensor) 
{ 
    sensors_.push_back(sensor); 
}


/************ GET SENSOR DATA CACHE ***********/

const std::vector<SensorData>& Robot::getSensorDataCache() const 
{ 
    return sensorDataCache_; 
}


/****************** SET MODE ******************/

void Robot::setMode(RobotMode mode) 
{ 
    mode_ = mode; 
}


/****************** GET MODE ******************/

RobotMode Robot::getMode() const 
{ 
    return mode_; 
}