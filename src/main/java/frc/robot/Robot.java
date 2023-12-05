// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PneumaticsTeleOp;
import frc.robot.subsystems.PneumaticsSubsytem;

public class Robot extends TimedRobot
{
  private final PneumaticsSubsytem m_PneumaticsSubsytem =
    new PneumaticsSubsytem();

  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    
  private Command slow;
  private Command med;
  private Command high;
  
  private Command m_autonomousCommand;
  private // auto chooser here
  

  @Override
  public void robotInit()
  {
    
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
    
  }

  @Override
  public void autonomousPeriodic()
  {

  }
  
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
