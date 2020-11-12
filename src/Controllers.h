/*
* @file      Controllers.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Controller Factory
*/
#ifndef __CONTROLLERS_H__
#define __CONTROLLERS_H__

// Included Libraries
#include <map>
#include <string>

// 3rd Party Libraries

// User Libraries

class ControllerInterface;

class ControllerFactory
{
    public:

	    // Enum containing the list of available controllers from the factory
        enum ControllerType
        {
            STANDARD_PID,
            SUPER_TWISTING_SMC
        };

        ControllerInterface *createController(std::string & controlType);

        static ControllerFactory *instance()
        {
            if (!_instance)
            {
                _instance = new ControllerFactory();
            }
            return _instance;
        }

    private:

	    explicit ControllerFactory();
        static ControllerFactory *_instance;
        std::map< std::string, enum ControllerType > _controllerNameToEnum;

};

class ControllerInterface
{
    public:
        virtual ~ControllerInterface() {}
        virtual void loadConfig(std::string & DOF) = 0;
        virtual void setTarget(float val) = 0;
        virtual void setStates(float val1, float val2) = 0;
        virtual void process(void) = 0;
        


};

#endif