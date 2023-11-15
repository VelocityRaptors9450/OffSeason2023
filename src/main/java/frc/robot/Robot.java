package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FL_wheel;
import frc.robot.commands.FR_wheel;
import frc.robot.commands.BL_wheel;
import frc.robot.commands.BR_wheel;

public class Robot extends TimedRobot {

  /* Command Declarations */
  private Command frontLeft;
  private Command frontRight;
  private Command backLeft;
  private Command backRight;

  /* Autonomous Chooser Declarations */
  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();


  Timer time = new Timer();
 
   @Override
   public void robotInit()
   {
    /* Initializing the Autonomous Chooser (stuff) */
    // Adds the options for the auto chooser.
    m_autoChooser.addOption("Front Left", frontLeft);
    m_autoChooser.addOption("Front Right", frontRight);
    m_autoChooser.addOption("Back Left", backLeft);
    m_autoChooser.addOption("Back Right", backRight);

    // Puts the auto chooser into it's own tab on Shuffleboard.
    ShuffleboardTab autoTab =
      Shuffleboard.getTab("Auto");

    Shuffleboard.getTab("Auto")
      .add("Autonomous Select:", m_autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withSize(3, 1);
   }
   

  @Override
  public void robotPeriodic() {
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
    time.restart();


    frontLeft = new FL_wheel().withTimeout(3);
    frontRight = new FR_wheel().withTimeout(3);
    backLeft = new BL_wheel().withTimeout(3);
    backRight = new BR_wheel().withTimeout(3);

    m_autonomousCommand = m_autoChooser.getSelected();

    //schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    time.restart();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }    
  }

  @Override
	public void teleopExit() {}
  
  int _loopCount = 0;
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}