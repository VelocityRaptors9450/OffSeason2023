// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  ShooterSubsystem shooter;
  private double targetVel;
  private double targetAcc;
  private double voltagefor;
  private double voltageback;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooter, double targetVel, double targetAcc, double voltagefor, double voltageback) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.voltagefor = voltagefor;
    this.voltageback = voltageback;
    this.targetVel = targetVel;
    this.targetAcc = targetAcc;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setVoltage(voltagefor, voltageback);
    //shooter.setVelocity(targetVel, targetAcc);
    SmartDashboard.putNumber("FrontSpinVelocity", shooter.getVelocityFrontSpin());
    SmartDashboard.putNumber("BackSpinVelocity", shooter.getVelocityBackSpin());  
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
