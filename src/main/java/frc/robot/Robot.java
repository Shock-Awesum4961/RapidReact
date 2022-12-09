// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*********************************************************************************************
 * 
 * TO DO:
 * Scale Down Limelight speed
 * 
 * 
 * 
 **********************************************************************************************/


 package frc.robot;

 import edu.wpi.first.wpilibj.Joystick;
 import edu.wpi.first.wpilibj.PneumaticsModuleType;
 import edu.wpi.first.wpilibj.TimedRobot;
 import edu.wpi.first.wpilibj.drive.MecanumDrive;
 import edu.wpi.first.wpilibj.livewindow.LiveWindow;
 import edu.wpi.first.wpilibj.Timer;
 //import edu.wpi.first.wpilibj.XboxController;
 import edu.wpi.first.wpilibj.GenericHID.RumbleType;
 import edu.wpi.first.wpilibj.DoubleSolenoid;
 import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
 
 import com.ctre.phoenix.motorcontrol.ControlMode;
 // phoenix
 import com.ctre.phoenix.motorcontrol.NeutralMode;
 import com.ctre.phoenix.motorcontrol.can.*;// <-- gets us access to WPI_TalonSRX which works with wpilibj.drive.Mecanum
 
 import com.revrobotics.CANSparkMax;
 import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 
 
 // limelight
 //import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; new comment
 import edu.wpi.first.cameraserver.CameraServer;
 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;
 
 //import java.lang.Math; new commment
 
 
 public class Robot extends TimedRobot 
 {
   // motors
   private static final int kFrontLeftChannel = 12;
   private static final int kRearLeftChannel = 14;
   private static final int kFrontRightChannel = 22;
   private static final int kRearRightChannel = 20;
   private static final int kFlywheelChannel = 10;// not put in yet
   private static final int kIntakeChannel = 4; // Intake Motor
 
   private static final int driverJoystickChannel = 0;
   private static final int operatorJoystickChannel = 1;
   
   // Set robot speed for various modes
   private static final double fastSpeed = 1.0;  // teleop hi speed
   private static final double slowSpeed = 0.4;  // teleop lo speed
   private static final double autoSpeed = 0.5; // autonomous speed
 
 // Set motor speeds for other motors
 private static final double intakeSpeed = 0.8;  // intake speed
 private static final double conveyorSpeed = 0.25; // conveyor speed
 
 
 private static final double flywheelSpeedFast = 1.0; // fast flywheel speed
 private static final double flywheelSpeedSlow = 0.55; // slow flywheel speed
 private /*static final*/ double flywheelSpeed = 1.0; // Initial flywheel speed
 
   
   private static final NeutralMode B_MODE = NeutralMode.Brake; // Set the talons neutralmode to brake
   private static final NeutralMode C_MODE = NeutralMode.Coast; // Set the talons neutralmode to coast
   private boolean modeAuto = false;
   private int autoStep = 0;  // step counter for auto shoot
   
   private MecanumDrive m_robotDrive;
   private Joystick driverJoystick;
   private Joystick operatorJoystick; 
 //  private XboxController driverJoystick;
 //  private XboxController operatorJoystick; 
 private final Timer l_timer = new Timer(); // timer for autonomous sequence
 private final Timer m_timer = new Timer(); // timer for manual shoot
 
   // Driver Buttons
   private static final int buttonSlowDrive = 5; // LB
   private static final int buttonFastDrive = 6; // RB
   private static final int buttonTargetSeek = 7; // Back/Select
 
   // Operator Buttons:
   //private static final int buttonIntakeUp = 1;  // A
   //private static final int buttonIntakeDown = 2;  // B
 //  private static final int buttonBypassLimelight = 3;  // X
 private static final int buttonIntakeToggle = 1;  // A
 private static final int buttonShoot = 4;  // Y
 private static final int buttonClimb1Toggle = 2;  // B
 private static final int buttonClimb2Toggle = 3;  // X
 
   // Limelight
   private PIDController rotationController = new PIDController(0.035, 0, 0);
   private PIDController distanceController = new PIDController(0.15, 0, 0);
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   NetworkTableEntry tv = table.getEntry("tv");
   NetworkTableEntry tx = table.getEntry("tx");
   NetworkTableEntry ty = table.getEntry("ty");
   NetworkTableEntry ta = table.getEntry("ta");
 //  private boolean shootdistanceOK = false;
 
   private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(kIntakeChannel);
 //  private WPI_TalonSRX flyWheelMotor = new WPI_TalonSRX(kFlywheelChannel);
   private WPI_TalonFX flyWheelMotor = new WPI_TalonFX(kFlywheelChannel);
 
   private CANSparkMax conveyorMotor;
 
   // Intake
   DoubleSolenoid intakeSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
   private boolean intakeDown = false;
   /*
   usage:
       intakeSol.set(kForward);
       intakeSol.set(kReverse); // raise intake
   */
 
   // Shoot
   DoubleSolenoid shootSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
 
   // Climb
   DoubleSolenoid climb1Sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
   private boolean climb1Up = false;
   DoubleSolenoid climb2Sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
   private boolean climb2Up = false;
 
   // State machine for Rumble
   static boolean state = false; // rumble state
   private final Timer r_timer = new Timer(); // Rumble timer
   private final Timer a_timer = new Timer(); // Auton shoot delay timer
 
   // State machine for Shoot
   static boolean state2 = false; // Shoot state
   static boolean state3 = false; // Intake state 
   static boolean state4 = false; // Climb1 state
   static boolean state5 = false; // Climb2 state 
   
 
 // end of data section
 
 /******************************************************* */
 /* This sections contains methods used by robot program */
 /******************************************************* */
   void autoShoot()  // called from autonomousPeriodic()
   {
     m_robotDrive.driveCartesian(0, 0, 0);  // Update drive in loop
     switch (autoStep)
     {
     case 0:
       a_timer.reset();
       flyWheelMotor.set(ControlMode.PercentOutput,flywheelSpeedFast);  // start flywheel
       autoStep = 1;
       break;
     case 1:
       if(a_timer.get() > 1.0) // pause before starting shoot
         {
         m_robotDrive.stopMotor(); // stop robot
         a_timer.reset();
         autoStep = 2;    
         }
         
       break;
 
     case 2:  // shoot!
 //      SmartDashboard.putString("Auto Step","Shoot!"); 
       shootBall(true);
       break;
 
     }
 
   }
 
 
   void doLimelight()
     {
       if (!(driverJoystick.getRawButton(buttonTargetSeek) || modeAuto))
         return;
 
     double rotateValue = 0;
     double driveValue = 0;
 
     // Limelight
     double targetValid = tv.getDouble(0.0);
 
     if (targetValid != 1.0) // back up until target is in sight
     {
       m_robotDrive.driveCartesian(1.0, 0, 0);
       return;
     }
 
     double x = tx.getDouble(0.0);
     double y = ty.getDouble(0.0);
     //  double area = ta.getDouble(0.0);
   
     //SmartDashboard.putNumber("Target Valid ", targetValid);
     //SmartDashboard.putNumber("Limelight X ", x);
     //SmartDashboard.putNumber("Limelight Y ", y);
     //  SmartDashboard.putNumber("Limelight Area ", area);
   
     rotateValue = rotationController.calculate(x, 0);
     driveValue = -distanceController.calculate(y, 0);
     //SmartDashboard.putNumber("rOutput", rotateValue);
     //SmartDashboard.putNumber("dOutput", driveValue);
 
     //  if ((driveValue < setpointOK) && (driveValue > -setpointOK))
 /*
     if (Math.abs(driveValue) < setpointOK)
       shootdistanceOK = true;
     else
       shootdistanceOK = false;
 */
     
     m_robotDrive.driveCartesian(driveValue, 0, rotateValue);
      
 
   /*
   if (!modeAuto) 
     {
 
     }
 */
   }
 
 
   @Override
   public void robotInit() 
   {
     LiveWindow.disableAllTelemetry();
     CameraServer.startAutomaticCapture();
 
     WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);//  
     WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);// 
     WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);// 
     WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);// 
 
     conveyorMotor = new CANSparkMax(1, MotorType.kBrushless);
     
     // Invert the right side motors.
     // You may need to change or remove this to match your robot.
     frontRight.setInverted(true); 
     rearRight.setInverted(true);
     frontLeft.setInverted(false);
     rearLeft.setInverted(false);
     
     // set neutral mode all motors
     frontLeft.setNeutralMode(B_MODE);
     rearLeft.setNeutralMode(B_MODE);
     frontRight.setNeutralMode(B_MODE);
     rearRight.setNeutralMode(B_MODE);
     flyWheelMotor.setNeutralMode(C_MODE);
     intakeMotor.setNeutralMode(C_MODE);
 
     m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
 
     m_robotDrive.setDeadband(0.1);
 
     driverJoystick = new Joystick(driverJoystickChannel);
     operatorJoystick = new Joystick(operatorJoystickChannel);
 //    driverJoystick = new XboxController(driverJoystickChannel);
 //    operatorJoystick = new XboxController(operatorJoystickChannel);
     
     rotationController.setTolerance(2);
     distanceController.setTolerance(2);
     r_timer.reset();
     r_timer.start();    
 
     //Config the ramp-up of flywheel motor
     flyWheelMotor.configOpenloopRamp(0.5);
   } 
 
 
   /** This function is run once each time the robot enters autonomous mode. */
   @Override
   public void autonomousInit() 
   {
     l_timer.reset();
     l_timer.start();
     a_timer.reset();
     a_timer.start();
     m_robotDrive.setMaxOutput(autoSpeed);
     modeAuto = true;
     autoStep = 0;
 //    shootdistanceOK = false;
     shootSol.set(kForward);  // release shoot solenoid
     state2 = false; // Shoot state
 
     /**SmartDashboard.putNumber("Target Valid ", 0);
     SmartDashboard.putNumber("Limelight X ", 0);
     SmartDashboard.putNumber("Limelight Y ", 0);
     
     SmartDashboard.putNumber("rOutput", 0);
     SmartDashboard.putNumber("dOutput", 0);
     SmartDashboard.putString("Auto Step","");
     **/
 
     conveyorMotor.set(conveyorSpeed);
 
   }
 
   /** This function is called periodically during autonomous. */
   // Limelight based autonomous
   @Override
   public void autonomousPeriodic() 
   {
   final double tmr = l_timer.get();
 
 //  if ((!shootdistanceOK) && (tmr < 8.0)) 
   if (tmr < 0.5) 
 //    doLimelight();
 m_robotDrive.driveCartesian(1.0, 0, 0);
 
   else
     autoShoot();
   }
 
 /*  time based autonomous
   @Override
   public void autonomousPeriodic() 
   {
   final double tm = m_timer.get();
     
     if (tm < 2.0) // step 1
       {
       m_robotDrive.driveCartesian(-0.5, 0, 0);
       }
     
     if ((tm >= 2.0) && (tm < 4.0)) // step 2
         {
         m_robotDrive.stopMotor(); // stop robot
         }
     
     if ((tm >= 4.0) && (tm < 6.0)) // step 3
         {
           m_robotDrive.setMaxOutput(autoSpeed*1.2);
         m_robotDrive.driveCartesian(0, 0, .5);
         }
       
       
      if (tm >= 6.0)  // step 4
         {
         m_robotDrive.stopMotor(); // stop robot
         }
 
   }
 */
   @Override
   public void teleopInit() 
   {
     modeAuto = false;
 //    shootdistanceOK = false;
     m_robotDrive.setMaxOutput(fastSpeed);
 
     l_timer.start();
     l_timer.reset();
     m_timer.start();
     m_timer.reset();
     intakeSol.set(kReverse); // raise intake
     intakeDown = false;
     climb1Up = false;
   
     conveyorMotor.set(conveyorSpeed);
     shootSol.set(kForward);  // release shoot solenoid
 
     /*
     SmartDashboard.putNumber("Target Valid ", 0);
     SmartDashboard.putNumber("Limelight X ", 0);
     SmartDashboard.putNumber("Limelight Y ", 0);
     
     SmartDashboard.putNumber("rOutput", 0);
     SmartDashboard.putNumber("dOutput", 0);
     SmartDashboard.putString("Auto Step",""); 
 */
  
   }
 
   @Override
   public void teleopPeriodic() 
   {
     // Use the joystick X axis for lateral movement, Y axis for forward
     // movement, and Z axis for rotation.
   /*
   Driver Joystick:
   Left Stick  - UP/DOWN = Forward/Reverse
               - LEFT/RIGHT = Steer (Rotate) Left/Right
   Right Stick - LEFT/RIGHT = Lateral Move Left/Right
   Back/Select Button - Locate Target using Limelight (Hold down until Robot Stops)
 
   Operator Joystick:
   A = Intake Up / toggle Intake
   B = Intake Down
   Y = Lift Ball to Shoot (initiates shoot sequence)
   X = Bypass limelight tracking while held down
   */
 
   if(driverJoystick.getRawButton(buttonTargetSeek)) // use joysticks if limelight is not seeking
     doLimelight();
   else
     m_robotDrive.driveCartesian(driverJoystick.getY(), -driverJoystick.getX(),  -driverJoystick.getRawAxis(4));
   
   if (driverJoystick.getRawButton(buttonSlowDrive))
       m_robotDrive.setMaxOutput(slowSpeed);
   if (driverJoystick.getRawButton(buttonFastDrive))
       m_robotDrive.setMaxOutput(fastSpeed);
 
 //  if (operatorJoystick.getRawButton(buttonBypassLimelight))
   //  shootdistanceOK = true;
 
 //  if(shootdistanceOK)
     //{
   if (operatorJoystick.getRawButton(buttonSlowDrive))
     flywheelSpeed = flywheelSpeedSlow;
   if (operatorJoystick.getRawButton(buttonFastDrive))
     flywheelSpeed = flywheelSpeedFast;
 
   flyWheelMotor.set(ControlMode.PercentOutput,flywheelSpeed);  // start flywheel
 
 //    if(rumble1(shootdistanceOK))
   shootBall(operatorJoystick.getRawButton(buttonShoot));
     //}
 
 
   // new intake logic (toggle)
   intakeMotor.set(ControlMode.PercentOutput,-intakeSpeed);
   toggleIntake(operatorJoystick.getRawButton(buttonIntakeToggle));
 
   /* old intake logic 
   if (operatorJoystick.getRawButton(buttonIntakeUp))
     intakeDown = false;
 
   if (operatorJoystick.getRawButton(buttonIntakeDown))
     intakeDown = true;
 
   if (intakeDown || operatorJoystick.getRawButton(buttonIntakeUp))
     {
     intakeMotor.set(ControlMode.PercentOutput,-intakeSpeed); 
     intakeSol.set(kForward); // lower intake
     }
   else
     {
 ///    intakeMotor.stopMotor();
     intakeMotor.set(ControlMode.PercentOutput,-intakeSpeed); 
     intakeSol.set(kReverse); // raise intake
     }
 */
 
 // Climb
   toggleClimb1(operatorJoystick.getRawButton(buttonClimb1Toggle));
   toggleClimb2(operatorJoystick.getRawButton(buttonClimb2Toggle));
 
   }
 
   // returns true after 1 sec of rumble  
   boolean rumble1(boolean event)
   {
   boolean finished = false;
 
   if (event  && !state)
     {
     r_timer.reset();
     operatorJoystick.setRumble(RumbleType.kLeftRumble, 0.6);
     state = true;
     }
    
   if (r_timer.get() > 1.0)
     {
     operatorJoystick.setRumble(RumbleType.kLeftRumble, 0.0);
     finished = true;
     }
    
   if (!event)
     state = false;
    
 //  SmartDashboard.putBoolean("state ", state);
 //  SmartDashboard.putNumber("tmr ", r_timer.get());
   return(finished);
 
   }
 
   // 
   // start flywheel and delay before calling shootBall()
   // Note shootTrigger signal must go low or set state2 = false before shooting again   
   void shootBall(boolean shootTrigger)
   {
    
   if (shootTrigger && !state2)  // 
     {
     m_timer.reset();
     shootSol.set(kReverse);
     state2 = true;
     }
    
   if (m_timer.get() > 0.5)
     shootSol.set(kForward);  // release shoot solenoid
    
   if (!shootTrigger)
     state2 = false;
      
   }
 
   void toggleIntake(boolean intakeTrigger)
   {
    
   if (intakeTrigger && !state3)  // 
     {
     if (intakeDown == false)
       {
       intakeSol.set(kForward); // lower intake
       intakeDown = true;
       }
     else
       {
       intakeSol.set(kReverse); // raise intake
       intakeDown = false;
       }
     state3 = true;
     }
 
     if (!intakeTrigger)
       state3 = false;
 
    }
       
    void toggleClimb1(boolean climb1Trigger)
    {
     
    if (climb1Trigger && !state4)  // 
      {
      if (climb1Up == false)
        {
        climb1Sol.set(kForward); // lower intake
        climb1Up = true;
        }
      else
        {
        climb1Sol.set(kReverse); // raise intake
        climb1Up = false;
        }
      state4 = true;
      }
  
      if (!climb1Trigger)
        state4 = false;
  
     }
        
     void toggleClimb2(boolean climb2Trigger)
     {
      
     if (climb2Trigger && !state5)  // 
       {
       if (climb2Up == false)
         {
         climb2Sol.set(kForward); // lower intake
         climb2Up = true;
         }
       else
         {
         climb2Sol.set(kReverse); // raise intake
         climb2Up = false;
         }
       state5 = true;
       }
   
       if (!climb2Trigger)
         state5 = false;
   
      }
         
            
   }
 