// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class AutoRotateCommand extends CommandBase {
  private final DriveTrain swerve;
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1. if the rate limit is 3
  // make bigger for sharper, make smaller for ramp/coast
  // decimals are fine, they will make the ramp/coast slower
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);
  
 
  public Timer t = new Timer();
  double time = 0.02;
  boolean ranOnce = false;
  DoubleSupplier rotation;
  DoubleSupplier heading;


  /** Creates a new RotateCommand, which allows rotation to a specified heading. */
  public AutoRotateCommand(DriveTrain swerve, DoubleSupplier heading, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    this.rotation = power;
    this.heading = heading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ranOnce) {
      time = t.get();
    } else {
      ranOnce = true;
    }

    swerve.rotateToHeading(heading.getAsDouble(), rotation.getAsDouble(), time);

    

    
    t.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
