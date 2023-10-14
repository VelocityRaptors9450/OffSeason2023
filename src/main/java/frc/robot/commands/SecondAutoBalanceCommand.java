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

public class SecondAutoBalanceCommand extends CommandBase {
  private final DriveTrain swerve;
  
  public Timer t = new Timer();
  double time = 0.02;
  boolean ranOnce = false;
  private boolean stop;
  private final double kp = 1;
 

  /** Creates a new ManualDriveCommand, which allows inputs from sources other than controllers */
  public SecondAutoBalanceCommand(DriveTrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(0, 0, 0, time);
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


    double angleError = swerve.getPitch();

    SmartDashboard.putNumber("Balance Angle Error", angleError);
    SmartDashboard.putNumber("Balance Power", angleError * kp);
    SmartDashboard.putString("Auto Balance", "Second");




    swerve.drive(0, angleError * kp, 0, time);





    

    
    t.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, time);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
