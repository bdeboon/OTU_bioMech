//////////////////////////////////////////////////////////
// ROSfalcon Simple Joystick Controller.
//
// Steven Martin
// 22/07/10

#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/UInt8.h"
#include "ros_falcon/falconSetPoint.h"


#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"


using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;
FalconDevice m_falconDevice;
FalconKinematicStamper falcon_legs;
std::array<double, 3> SetPoint, Pos, Error, prevError, dError, integral, forces, Forces;
boost::array<float, 3> KpGain, KiGain, KdGain;

/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/

bool init_falcon(int NoFalcon)

{
    cout << "Setting up LibUSB" << endl;
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
    m_falconDevice.setFalconGrip<FalconGripFourButton>(); //Set Grip
    if(!m_falconDevice.open(NoFalcon)) //Open falcon @ index
    {
        cout << "Failed to find falcon" << endl;
        return false;
    }
    else
    {
        cout << "Falcon Found" << endl;
    }

    //There's only one kind of firmware right now, so automatically set that.
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();
    //Next load the firmware to the device

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    firmware_loaded = m_falconDevice.isFirmwareLoaded();
    if(!firmware_loaded)
    {
        cout << "Loading firmware" << endl;
        uint8_t* firmware_block;
        long firmware_size;
        {

            firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
            firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


            for(int i = 0; i < 20; ++i)
            {
                if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

                {
                    cout << "Firmware loading try failed" <<endl;
                }
                else
                {
                    firmware_loaded = true;
                    break;
                }
            }
        }
    }
    else if(!firmware_loaded)
    {
        cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
        return false;
    }
    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue" << endl;
        return false;
    }
    cout << "Firmware loaded" << endl;

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set" << endl;
    std::array<int, 3> forces;
    //m_falconDevice.getFalconFirmware()->setForces(forces);
    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    bool homing_reset = false;
    usleep(100000);
    int tryLoad = 0;
    while(!stop) //&& tryLoad < 100)
    {
        if(!m_falconDevice.runIOLoop()) continue;
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;

            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
            cout << "Falcon homed." << endl;
            homing_reset = true;
            stop = true;
        }
        tryLoad++;
    }
    /*if(tryLoad >= 100)
    {
        return false;
    }*/

    m_falconDevice.runIOLoop();
    return true;
}
void get_setpoint(const ros_falcon::falconSetPoint::ConstPtr& point){
  std::array<double, 3> coords, LegAngles;
  coords[0] = point->X;
  coords[1] = point->Y;
  coords[2] = point->Z;

  //Convert cartesian coordinates to leg angles to determine validity
falcon_legs.getAngles(coords, LegAngles);
for(int i = 0; i < 3; i++)
  {
      if(std::isnan(LegAngles[i]))
      {
         ROS_ERROR("Requested coordinates not in Falcon workspace");
         return;
      }
  }


 for(int i = 0; i < 3; i++)
  {

    //Store falcon setpoint
    SetPoint[i] = coords[i];

  }

}

void runPID(){
  //Request the current positon:
  Pos = m_falconDevice.getPosition();


      ROS_DEBUG("Position= %f %f %f",Pos[0], Pos[1], Pos[2]);


//DETERMINE CURRENT ERROR, DELTA ERROR AND INTEGRAL
for(int i = 0; i < 3; i++)
{
  Error[i] = SetPoint[i] - Pos[i];
  dError[i] = Error[i] - prevError[i];
  integral[i] += Error[i];

  //Integral anti-windup
          if(integral[i] > 5)
              integral[i] = 5;

          if(integral[i] < -5)
              integral[i] = -5;

}
      ROS_DEBUG("X Error= %f Y Error = %f Z Error = %f",Error[0], Error[1], Error[2]);
      ROS_DEBUG("X dError= %f Y dError = %f Z dError = %f",dError[0], dError[1], dError[2]);
      ROS_INFO("X Integral= %f Y Integral = %f Z Integral = %f",integral[0], integral[1], integral[2]);

      //Simple PID controller. Repeated for each axis.
for(int i = 0; i < 3; i++)
{
  Forces[i]= KpGain[i]*Error[i] + KiGain[i]*integral[i] + KdGain[i]*dError[i];

}

  //  m_falconDevice.setForce(forces); //Write falcon forces to driver (Forces updated on IO loop) Should run @ ~1kHz

    prevError = Error;	//Store current error for comparison on next loop
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv, "FalconJoystick");

    ros::NodeHandle node;
    int falcon_int;
    bool debug;
    //Button 4 is mimic-ing clutch.
    bool clutchPressed, coagPressed;
    node.param<int>("falcon_number", falcon_int, 0);
    node.param<bool>("falcon_debug", debug, false);
    node.param<bool>("falcon_clutch", clutchPressed, true);
    node.param<bool>("falcon_coag", coagPressed, true);

    if(init_falcon(falcon_int))
    {
        cout << "Falcon Initialised Starting ROS Node" << endl;

        m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

        ros::Subscriber feedback_control_sub = node.subscribe<ros_falcon::falconSetPoint>("/slave_to_master", 1000, get_setpoint);

        //Start ROS Publisher
        ros::Publisher pub = node.advertise<ros_falcon::falconSetPoint>("/master_to_slave",1000);
        ros::Rate loop_rate(1000);

        while(node.ok())
        {
            sensor_msgs::Joy Joystick;
            ros_falcon::falconSetPoint position;
            std::array<double, 3> prevPos;

            Joystick.buttons.resize(1);
          //  Joystick.axes.resize(3);

            std::array<double,3> forces;
            //Request the current encoder positions:
            std::array<double, 3> Pos;
            std::array<double, 3> newHome, prevHome;
            int buttons;

            if(m_falconDevice.runIOLoop())
            {
                ros::spinOnce();
                /////////////////////////////////////////////
                Pos = m_falconDevice.getPosition();  //Read in cartesian position

                buttons = m_falconDevice.getFalconGrip()->getDigitalInputs(); //Read in buttons

                //Publish ROS values
                Joystick.buttons[0] = buttons;
                //Joystick.axes[0] = Pos[0];
                //Joystick.axes[1] = Pos[1];
                //Joystick.axes[2] = Pos[2];
                position.X = Pos[0];
                position.Y = Pos[1];
                position.Z = Pos[2];
                pub.publish(position);

                //GAIN VALUES

                  KpGain[0] = 110;        //X axis Proportional gain. Ku: 185, Period of Oscillation at Ku: 166ms (Recommended 110)
                  KiGain[0] = 0.3;      //X axis Integral gain. (Recommended 0.3)
                  KdGain[0] = 2303;    //X axis Derivative gain. (Recommended 2303)

                  KpGain[1] = 110;        //Y axis Proportional gain. Ku: 185, Period of Oscillation at Ku: 166ms (Recommended 110)
                  KiGain[1] = 0.3;      //Y axis Integral gain. (Recommended 0.3)
                  KdGain[1] = 2303;    //Y axis Derivative gain. (Recommended 2303)

                  KpGain[2] = 111;    //Z axis Proportional gain. Ku: 200, Period of Oscillation at Ku: 300ms (Recommended 111)
                  KiGain[2] = 0.2;    //Z axis Integral gain. (Recommended 0.2)
                  KdGain[2] = 4500;   //Z axis Derivative gain. (Recommended 4500)

                /*  //Default location (Falcon will move to this position following homing)
                  SetPoint[0] = 0.0;    //-0.065 <= X <= 0.065
                  SetPoint[1] = 0.0;    //-0.065 <= Y <= 0.065
                  SetPoint[2] = 0.075; //0.0 <= Z <= 0.175
                  */
                 //TODO if Joystick can subscribe to twist message use those forces instead for haptic feedback
                //if
                  float KpGainX = -200;
                  float KpGainY = -200;
                  float KpGainZ = -200;

                  float KdGainX = -500;
                  float KdGainY = -500;
                  float KdGainZ = -500;

                  // Check if button 4 is pressed, set the forces equal to 0.
                  if(buttons == 4 || buttons == 2){
                      if(buttons == 4 && coagPressed == false){
                          ROS_INFO("Coag Pressed (Button 4)");
                          coagPressed = true;
                      }
                      else if(buttons == 2 && clutchPressed == false){
                          ROS_INFO("Clutch Pressed (Button 2)");
                          clutchPressed = true;
                      }

                      forces[0] = ((Pos[0] - newHome[0]) * KpGainX) + (Pos[0] - prevPos[0])*KdGainX;
                      forces[1] = ((Pos[1] - newHome[1]) * KpGainY) + (Pos[1] - prevPos[1])*KdGainY;
                      forces[2] = ((Pos[2] - newHome[2]) * KpGainZ) + (Pos[2] - prevPos[2])*KdGainZ;
                  }
                  else{

                    if(coagPressed == true){
                        ROS_INFO("Coag Released (Button 4)");
                        coagPressed = false;
                        newHome = Pos;
                    }
                    else if(clutchPressed == true){
                        ROS_INFO("Clutch Released (Button 2)");
                        clutchPressed = false;
                        newHome = Pos;
                    }
                    runPID();
                    forces[0] = Forces[0];
                    forces[1] = Forces[1];
                    forces[2] = Forces[2];

                    }
                m_falconDevice.setForce(forces); //Write falcon forces to driver (Forces updated on IO loop) Should run @ ~1kHz


                if(debug)
                {
                    cout << "Position= " << Pos[0] <<" " << Pos[1] << " " << Pos[2] <<  endl;
                    cout << "newHome  = " << newHome[0] <<" " << newHome[1] << " " << newHome[2] <<  endl;
                    cout << "Error   =" << Pos[0] - newHome[0] <<" " << Pos[1]-newHome[1] << " " << Pos[2] -newHome[2] <<  endl;
                    //cout << "Force= " << forces[0] <<" " << forces[1] << " " << forces[2] <<  endl;
                }
                prevPos = Pos;
                prevHome = newHome;
            }
            loop_rate.sleep();
        }
        m_falconDevice.close();
    }
    return 0;
}
