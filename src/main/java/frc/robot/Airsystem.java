/*

______                    ______       _        ______                                      _  ______ _ _ _       
| ___ \                   | ___ \     | |       | ___ \                                    | | | ___ \ (_) |      
| |_/ /_ __ __ ___   _____| |_/ / ___ | |_ ___  | |_/ / __ _ _ __ _ __  _   _  __ _ _ __ __| | | |_/ / |_| |_ ____
| ___ \ '__/ _` \ \ / / _ \ ___ \/ _ \| __/ __| | ___ \/ _` | '__| '_ \| | | |/ _` | '__/ _` | | ___ \ | | __|_  /
| |_/ / | | (_| |\ V /  __/ |_/ / (_) | |_\__ \ | |_/ / (_| | |  | | | | |_| | (_| | | | (_| | | |_/ / | | |_ / / 
\____/|_|  \__,_| \_/ \___\____/ \___/ \__|___/ \____/ \__,_|_|  |_| |_|\__, |\__,_|_|  \__,_| \____/|_|_|\__/___|
                                                                         __/ |                                    
                                                                        |___/                            
*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.*;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS.SerialDataType;



public class Airsystem 
{
    private Joystick joyl;
    private Joystick joyr;
    private Solenoid solonoid;
    private Compressor compressor;


    public Airsystem()
    {
        //Define the Solonoid and Compressor to round off the Airsystem
        joyl = new Joystick(0);
        joyr = new Joystick(1);
        solonoid = new Solenoid(1);

        //Start the compressor
        compressor = new Compressor(0);
        compressor.start();

        //Announce activation
        System.out.println("Airsystem Activated!");

    
    }

    public void activateGrabber()
    {
        
        if(joyl.getTrigger() == true){

            solonoid.set(true);

        }
        else{
           solonoid.set(false);
        }
    }
    public void autoGrabber()
    {
       

    }
       


    }


