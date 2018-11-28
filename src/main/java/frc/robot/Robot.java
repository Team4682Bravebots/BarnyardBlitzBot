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
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.XboxController;

 
public class Robot extends IterativeRobot {


//Define the Joysticks
//Comment out and comment in the control scheme you want to use.
Joystick JoyL = new Joystick(0); //Left Joystick
Joystick JoyR = new Joystick(1); //Right Joystick

//Define joystick speeds
double JoyLY;
double JoyRY;

//Initialize the Airsystem
Airsystem pneumatics;

//Define the xbox speeds
double xboxSpeed;

//Define the Motors
TalonSRX leftT;//Left
TalonSRX rightT; //Right
TalonSRX leftB;
TalonSRX rightB;
TalonSRX arm; // Arm motor



//Define the Gyro
AHRS ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);

//Define PID
PIDController armPID = new PIDController(0.0, 0.0, 0.0, 0.0, ahrs, arm);

//Define the Timer for auto
Timer timer = new Timer();

///@param lspeed
///@param rspeed
//Tank drive method


  public void teleopDrive(double lspeed, double rspeed)
  {
    
    leftT.set(ControlMode.PercentOutput, lspeedt);
    rightT.set(ControlMode.PercentOutput, rspeed);
    leftB.set(ControlMode.PercentOutput, lspeed);
    rightB.set(ControlMode.PercentOutput, rspeed);

  }
  ///@param desiredDegrees: Setpoint for PID controller
  ///@param armSpeed: Average speed to set the arm. Controlled with joystick
  //Control method for the arm
  public void controlArm()
  {
    timer.start();
    while(isAutonomous() && isEnabled())
    {
      if(timer.get() < 1.5){
       pneumatics.activateGrabber();
       arm.set(ControlMode.PercentOutput, 0.5)
      }
      timer.delay(5);
    }
    timer.stop();
    pneumatics.deactivateGrabber();
     
  }


@Override
  public void robotInit() 
  {
  leftT = new TalonSRX(0); 
  rightT= new TalonSRX(1);
  leftB = new TalonSRX(3);
  rightB = new TalonSRX(4);
  arm = new TalonSRX(2);
  pneumatics = new Airsystem();
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
    JoyLY = JoyL.getY();
    JoyRY = JoyR.getY();

    teleopDrive(JoyLY, JoyRY);

    if(JoyL.getTrigger() == true)
    {
      pneumatics.activateGrabber();

    }
    elif(JoyL.getTrigger() == false)
    {
      pneumatics.deactivateGrabber();
    }

  }

  @Override
  public void testPeriodic() 
  {
  }
}
