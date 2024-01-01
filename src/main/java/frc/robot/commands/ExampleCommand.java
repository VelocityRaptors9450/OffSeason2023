//5) Now that you're here, lets learn about commands, Go through all the comments and see what each part is for.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  //variables for the Command
  boolean isFinished = false;
  private final RomiDrivetrain subsystem;

  //This methods helps define variables used in the command by taking in parameters 
  public ExampleCommand(RomiDrivetrain subsystem) {
    this.subsystem = subsystem;// this will set the command variable subsystem to the parameter subsystem

    //This line just makes sure 2 commands don't run something from the same subsystem
    addRequirements(subsystem);
  }

  // This method is called when the command is initially scheduled in the schedualer 
  @Override
  public void initialize() {}

  // This method starts being called in a loop when the scheduler runs the command (this is where the main code goes)
  @Override
  public void execute() {}

  // This method is called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // when this method returns true, it will end the commad
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  /*Now use the WPILIB command pallet to create a new command called autoSqaure.
  This command should make the romi go in a square with each sidelength being 12 inches. Use PIDs and ecoders to create the method
  but you must not use a controller, then after you are some creating a draft method, Ask another programer 
  to teach you how to schedule commands. Then test it in the autonomousInit() method in Robot.java. After revision and testing check 
  with another and get their approval before moving on to the next comment
  */

  //Now go to back to README.md and SCROLL TO THE BOTTOM
}
