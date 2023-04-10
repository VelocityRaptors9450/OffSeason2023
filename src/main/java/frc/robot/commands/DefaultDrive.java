// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveTest;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  DriveTrain driveTrain;
 
  //private SwerveTest swerveDrive;
  private CommandXboxController cont;
  public DefaultDrive(DriveTrain drive, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    //swerveDrive = sw;
    
    cont = controller;
    driveTrain = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fwd = cont.getLeftY();
    double strafe = cont.getLeftX();
    double rot = cont.getRightX();
    if(rot != 0){
      
    }else if(fwd != 0){
      driveTrain.rotateToZero();
    }else{
      driveTrain.drive(fwd, strafe, rot);
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
