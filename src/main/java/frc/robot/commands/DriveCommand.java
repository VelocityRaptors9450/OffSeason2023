// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.BooleanSupplier;
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

public class DriveCommand extends CommandBase {
  private final DriveTrain swerve;
  private final CommandXboxController controller;
  //private DoubleSupplier rightX, leftX, leftY;
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1. if the rate limit is 3
  // make bigger for sharper, make smaller for ramp/coast
  // decimals are fine, they will make the ramp/coast slower
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);
  
 
  public Timer t = new Timer();
  double time = 0.02;
  boolean ranOnce = false;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrain swerve, CommandXboxController controller/*DoubleSupplier rightX, DoubleSupplier leftX, DoubleSupplier leftY*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.leftX = leftX;
    // this.rightX = rightX;
    // this.leftY = leftY;
    this.controller = controller;

    this.swerve = swerve;
    addRequirements(swerve);
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

   
    // if(controller.getHID().getBButtonPressed()){
    //   swerve.fieldRelativeSwitch();
    // }


    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02)) * swerve.kMaxSpeed;
    //final double xSpeed = -controller.getLeftX() * swerve.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02)) * swerve.kMaxSpeed;
    //final double ySpeed = controller.getLeftY() * swerve.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02)) * swerve.kMaxAngularSpeed;
    // final double rot = controller.getRightX() * swerve.kMaxAngularSpeed;
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);

    swerve.drive(xSpeed, ySpeed, rot, time);

    

    
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
