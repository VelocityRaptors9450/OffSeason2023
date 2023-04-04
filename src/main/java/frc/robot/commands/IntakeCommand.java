// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import frc.robot.subsystems.IntakeSubsystem;

// public class IntakeCommand extends CommandBase {
  
//   private final IntakeSubsystem subsystem;
//   /** Creates a new IntakeCommand. */
//   public IntakeCommand(IntakeSubsystem subsystem) {
//     this.subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("Intake started");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     subsystem.intake(.4);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     subsystem.setMotorStop();
//     System.out.println("Intake stopped");

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
