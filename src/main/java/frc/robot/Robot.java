// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PneumaticsAuto;
import frc.robot.commands.PneumaticsTeleOp;
import frc.robot.subsystems.PneumaticsSubsystem;

public class Robot extends TimedRobot
{
  private final PneumaticsSubsystem m_PneumaticsSubsytem =
    new PneumaticsSubsystem();

  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    
  private Command slow;
  private Command med;
  private Command high;
  
  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  

  @Override
  public void robotInit()
  {
    slow = new PneumaticsAuto(m_PneumaticsSubsytem, driverController, 8).withTimeout(20);
    med = new PneumaticsAuto(m_PneumaticsSubsytem, driverController, 6).withTimeout(20);
    high = new PneumaticsAuto(m_PneumaticsSubsytem, driverController, 4).withTimeout(20);

    m_autoChooser.addOption("slow", slow);
    m_autoChooser.addOption("medium", med);
    m_autoChooser.addOption("high", high);

    ShuffleboardTab autoTab =
      Shuffleboard.getTab("Auto");

    Shuffleboard.getTab("Auto")
      .add("Autonomous Select:", m_autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withSize(4, 1);
  }

  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  Timer t = new Timer();
  @Override
  public void autonomousInit()
  {
    m_autonomousCommand = m_autoChooser.getSelected();
  }

  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopInit()
  {
    m_PneumaticsSubsytem.setDefaultCommand(new PneumaticsTeleOp(m_PneumaticsSubsytem, driverController));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

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
