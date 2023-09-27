// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.AutoRotateCommand;
import frc.robot.subsystems.ArmSubsystem;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  

  private static Robot instance = null;
  //private final PowerDistribution PDP = new PowerDistribution(1, ModuleType.kRev);


  public static Robot getInstance(){
    if(instance == null) instance = new Robot();
    return instance;
  }

  public double getVoltage(){
    return /*PDP.getVoltage()*/ 12;
  }

  
  
  

  
  //private ShooterSubsystem shooter = new ShooterSubsystem();
  //private CANSparkMax leftMotor1 = new CANSparkMax(6, MotorType.kBrushless);
  //private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  //private CANSparkMax rightMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  //private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

  //private Joystick joy1 = new Joystick(0);



  private Command m_autonomousCommand;
  Timer time = new Timer();
  //private TalonFX motor1 = new TalonFX(1);
  
  //private DoubleLogEntry telemetry;
  private RobotContainer m_robotContainer;
 // private Timer time = new Timer();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //motor1.set(ControlMode.PercentOutput, 0); 

    //inverting directions
    //leftMotor1.setInverted(true);
    //rightMotor1.setInverted(true);

    // "slave" settings
    /* 
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor2.setInverted(true);
    rightMotor2.setInverted(true);
*/
    //init encoders(only init if want ot reset)
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("DrivePower", leftMotor1.getEncoder().getPosition() * Constants.ModuleConversion.drivetcks2ftfactor);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    


  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  
  @Override
  public void autonomousInit() {
    time.restart();
    //m_autonomousCommand = new ManualDriveCommand(m_robotContainer.driveTrain, () -> 0, () -> 0, () -> 15).withTimeout(5)
    //.andThen(new ManualDriveCommand(m_robotContainer.driveTrain, () -> 0, () -> 5, () -> 0).withTimeout(3))
    //.andThen(new ManualDriveCommand(m_robotContainer.driveTrain, () -> 0, () -> 0, () -> 10).withTimeout(7));

    m_autonomousCommand = new AutoRotateCommand(m_robotContainer.driveTrain, () -> 40, () -> 5).withTimeout(10)
    .andThen(new AutoRotateCommand(m_robotContainer.driveTrain, () -> 280, () -> 5).withTimeout(10));

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //System.out.println("Here");
    


    


  }
  //private CANSparkMax test = new CANSparkMax(9, MotorType.kBrushless);
  @Override
  public void teleopInit() {
    time.restart();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    


    // ExampleSubsystem.t.restart();
    // ExampleSubsystem.l.restart();
    // // ExampleSubsystem.motor6.getEncoder().setPosition(0);
    // ExampleSubsystem.intakeRight.getEncoder().setPosition(0);

    // ExampleSubsystem.intakeLeft.getEncoder().setPosition(0);

    // ExampleSubsystem.g.restart();
    
   // time.restart();
    //motor1.setSelectedSensorPosition(0);
   // DataLogManager.start();
    //DataLog log = DataLogManager.getLog();
    //telemetry = new DoubleLogEntry(log, "/my/double");

    
  }
  //CANCoder angle = new CANCoder(1);
  /** This function is called periodically during operator control. */

  
  int _loopCount = 0;
  @Override
  public void teleopPeriodic() {

    
    

    // System.out.println(cancoder.getPosition());
    // System.out.println(cancoder.getAbsolutePosition());
    // System.out.println();
    // System.out.println(cancoder1.getPosition());
    // System.out.println(cancoder1.getAbsolutePosition());
    // System.out.println();
    // System.out.println(cancoder2.getPosition());
    // System.out.println(cancoder2.getAbsolutePosition());
    // System.out.println();
    // System.out.println(cancoder3.getPosition());
    // System.out.println(cancoder3.getAbsolutePosition());
    // System.out.println();
    // System.out.println();
    

    
    

    
    

    //2048 tics / revolution
    /*
     while(motor1.getSelectedSensorPosition() < 5000){
      motor1.set(ControlMode.PercentOutput, 0.2);
      telemetry.append(motor1.getSelectedSensorPosition());
    }
    motor1.set(ControlMode.PercentOutput, 0);
    */
    /* 
    while(motor2.getEncoder().getPosition() < 42){
      motor2.set(0.2);
      System.out.println(motor2.getEncoder().getPosition());
    }
    motor2.set(0);
    */
    /*
     
    while(angle.getPosition() < 180){
      motor1.set(ControlMode.PercentOutput, 0.2);
      System.out.println(angle.getPosition());
    }
    motor1.set(ControlMode.PercentOutput, 0);
    */



  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    //CommandScheduler.getInstance().cancelAll();
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // tx = table.getEntry("tx");
    // ty = table.getEntry("ty");
    // ta = table.getEntry("ta");
    // tv = table.getEntry("tv");
    // test = table.getEntry("targetpose_cameraspace");


    


    // table.getEntry("ledMode").setNumber(3);
    // time.start();
    // time.reset();
    

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    
    
    // double x = tx.getDouble(0);
    // double y = ty.getDouble(0);
    // double area = ta.getDouble(0);
    // double haveTarget = tv.getDouble(0);
    // double target = 0;
    
    //System.out.println("LimelightX: " + x);
    //System.out.println("LimelightY: "+ y);
    
    //System.out.println("LimelightHasTarget: " + haveTarget);
    // System.out.println("Testing Pose 0: " + pose[0]);
    // System.out.println("Testing Pose 1: " + pose[1]);
    // System.out.println("Testing Pose 2: " + pose[2]);
    // System.out.println("Testing Pose 3: " + pose[3]);
    // System.out.println("Testing Pose 4: " + pose[4]);
    // System.out.println("Testing Pose 5: " + pose[5]);




    // if(area < 1.65 && haveTarget != 0){
    //   m_robotContainer.swerve.setDrivePower(0.1);
     
      

      
    // }else if(area > 1.9 && haveTarget != 0){
    //   m_robotContainer.swerve.setDrivePower(-0.1);
      

      
    // }else if(area < 1.9 && area > 1.65 && haveTarget != 0){
    //   if(x < -3){
    //     x = 90;
    //     m_robotContainer.swerve.setDrivePower(0.1);


    //   }else if(x > 3){
    //     x = 90;
    //     m_robotContainer.swerve.setDrivePower(-0.1);

    //   }

    // }else{
    //   m_robotContainer.swerve.setDrivePower(0);
      
    // }

    

    // System.out.println("LimelightAngle: " + (-x));

    // m_robotContainer.swerve.pid(Math.toRadians(-x), Math.toRadians(-x), Math.toRadians(-x), Math.toRadians(-x), 0.3);


    // if(time.get() < 1){
    //   motor.set(0.9);
    //   System.out.println("Out");
    // }else if(time.get() < 1){
    //   motor.set(-0.3);
    //   System.out.println("In");
    // }else{
    //   motor.set(0);
    //   System.out.println("Done");
    // }





  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
