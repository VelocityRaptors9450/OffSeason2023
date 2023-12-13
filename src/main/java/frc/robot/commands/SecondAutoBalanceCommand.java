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
import frc.robot.subsystems.DrivebaseSubsystem;

public class SecondAutoBalanceCommand extends CommandBase {
  private final DrivebaseSubsystem swerve;
  
  public Timer t = new Timer();
  double time = 0.02;
  boolean ranOnce = false;
  private boolean stop;
  private final double kp = 1.1;
 

  /** Creates a new ManualDriveCommand, which allows inputs from sources other than controllers */
  public SecondAutoBalanceCommand(DrivebaseSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerve.drive(0, 0, 0, time);
    stop = false;
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ranOnce) {
      time = t.get();
    } else {
      ranOnce = true;
    }


    // double anglePower = swerve.getPitch() * kp;

    
    SmartDashboard.putString("Auto Balance", "Second");

    // anglePower = MathUtil.clamp(anglePower, -5, 5);


    // swerve.drive(0, anglePower, 0, time);

    // if(swerve.getPitch() < 2){
    //   stop = true;
    // }





    

    
    t.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // swerve.drive(0, 0, 0.01, time);

    //Not sure if we need this, but just giving enough time for wheels to get to X position
    t.reset();
    while(t.get() < 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
