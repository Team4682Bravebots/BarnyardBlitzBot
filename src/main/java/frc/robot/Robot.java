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

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import jaci.pathfinder.*;
import edu.wpi.first.wpilibj.PIDController;

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
import com.kauailabs.navx.frc.AHRS.SerialDataType;

public class Robot extends IterativeRobot {

//Define the Joysticks
Joystick JoyL = new Joystick(0); //Left Joystick
Joystick JoyR = new Joystick(1); //Right Joystick

//Define joystick speeds
double JoyLY;
double JoyRY;

//Define the Motors
TalonSRX fl = new TalonSRX(0); //Front left
TalonSRX fr = new TalonSRX(1); //Front Right
TalonSRX bl = new TalonSRX(2); //Back Left
TalonSRX br = new TalonSRX(3); // Back Right




//Define the Gyro
AHRS ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);



//tankDrive() >>>>>>>>>>>>>>>> dt()

///@param fls front left motor speed
///@param frs front right motor speed
///@param bls back left motor speed
///@param brs back right motor speed
  public void tankDrive(double FLS, double FRS, double BLS, double BRS)
  {
    
    fl.set(ControlMode.PercentOutput, FLS);
    fr.set(ControlMode.PercentOutput, FRS);
    bl.set(ControlMode.PercentOutput, BLS);
    br.set(ControlMode.PercentOutput, BRS);


  }

///@param fls front left motor speed
///@param frs front right motor speed
///@param bls back left motor speed
///@param brs back right motor speed
//Autonomous drive command
  public void autoDrive(double FLS, double FRS, double BLS, double BRS)
  {
   
  }

@Override
  public void robotInit() 
  {
   

  }


  @Override
  public void robotPeriodic() 
  {

  }


  @Override
  public void autonomousInit() 
  {

  }

  @Override
  public void autonomousPeriodic()
  {

  }

  @Override
  public void teleopPeriodic() 
  {
  

  }

  @Override
  public void testPeriodic() 
  {
  }
}
