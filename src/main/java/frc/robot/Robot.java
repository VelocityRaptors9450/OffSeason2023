
// 2) now that you're here, go down below the the next comment you see

//4) now that you are here AGAIN, go down to the robotInit() method


package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// 2) You will need to import this to get access the RomiDrivetrain file. Now go to the next comment you see.
import frc.robot.subsystems.RomiDrivetrain;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer Container;
  /*  2) Now we have to get the functions from the RomiDriveTrain.java this is the code.
  This code is written for you this once, but you from now on you will have to write these *objects* yourselves. This is the syntax for objects:
  public <name of file> <variable name> = new <name of file>();    */
  public RomiDrivetrain drive = new RomiDrivetrain();
  /*2) The Robot.java file is the main program with it being the program that runs when we actually simulate the code.
  To run the program turn on the romi and connect to the WI-FI then press ctrl+shift+p (cmd+shift+p on mac) to open the command pallete then type 
  "simulate" and select "WPILIB: Simulate Robot Code". After proccessing for a bit A new simulate window will open with all 
  the things needed to control the robot such as the different modes, controllers, & controller statistics*/

  //3) Now go to RobotContainer.java for the next part of the program




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* 4) Now that you're here we are going to learn about PIDs
    first watch this video using this link to gain an understanding of PIDs
    https://www.youtube.com/watch?v=jIKBWO7ps0w 
    If you have any questions ask another programer

    Now create a new PIDcontroller object called pid and in the () put the three doubles kp , ki and kd
    Write the new PIDcontroller object below.
    */
    

    //4) after you're done go to the autonomousPeriodic()
    

     



    Container = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //4) Now that you're here, we are going to make the Romi go exactly 1 foot. To do this follow the steps below
    //Use the PID calculate function to calculated speed needed
    //pid.calculate( encodervalue(only get one side using a drive_function), double distance_needed )
    //Then apply that speed to both wheels using the drive.go function.
    //write it down below

  
    /*4) Now that you did this run the program and see if it goes 12 inches before stoping
    Now that you see that it DIDN'T go 12 inches, you have to go back to the PID values and cahnge the values.
    You will have to keep doing this until it is really close to 12 inches and doesn't occilate much, (I recomend printing the encoder values)
    After you got it close CONSISTANTLY check with one of the other programers to see if it is close eneough. if they say yes, move 
    onto the next section. One thing you should know before moving on though is that you will soon have to learn feadForwards,
    these are used to counteract external forces like friction that may hurt PIDs performance*/

    //5) Now go to the ExamnpleCommand.java file to learn Commands


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
