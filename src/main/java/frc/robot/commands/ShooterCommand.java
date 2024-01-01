// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LimelightTurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  ShooterSubsystem shooter;
  LimelightTurretSubsystem limeTurret;
  private double targetVel;
  private double targetAcc;
  private boolean leftBumper;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooter, LimelightTurretSubsystem limeTurret, boolean leftBumper, double targetVel, double targetAcc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.limeTurret = limeTurret;
    
    this.leftBumper = leftBumper;

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
    //shooter.setVelocity(targetVel, targetAcc);
    if(leftBumper){
      shooter.shootToPos(limeTurret.getHasTarget() ? limeTurret.getDistance() : 0);
    }else{
      shooter.setVelocity(0, 0);
    }
     
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
