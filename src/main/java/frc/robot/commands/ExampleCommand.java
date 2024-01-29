/*5) Now that you're here, let's learn about commands, Go through all the comments and see what each part is for. 
Stop for the next once you reach the bottom of the code.
*/ 

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  //Variables for the Command
  boolean isFinished = false;
  private final RomiDrivetrain subsystem;

  //This method helps define variables used in the command by taking in parameters 
  public ExampleCommand(RomiDrivetrain subsystem) {
    this.subsystem = subsystem;// this will set the command variable subsystem to the parameter subsystem
    
    //This line just makes sure 2 commands don't run something from the same subsystem
    addRequirements(subsystem);
  }

  // This method is called when the command is initially scheduled in the scheduler 
  @Override
  public void initialize() {}

  // This method starts being called in a loop when the scheduler runs the command (this is where the main code goes)
  @Override
  public void execute() {}

  // This method is called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // When this method returns true, it will end the command
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  /*Now use the WPILIB command pallet to create a new command called autoSqaure. This command should already have all the methods shown here.
  This command should make the romi go in a square with each side length being 12 inches. Use PIDs and encoders to create the command
  but you must not use a controller. 
  To schedule a command (aka to run), first, create an object for the file.
  Then, go to the place you want the command to run 
  (don't put it in a periodic method cause then you will call the command again before you finish the command causing problems, I suggest in the 
  autonomousInit() method)
  Finally type <object name>.schedule(); ex:exCommand.schedule.
  *****Remember to come back here afterward.*****
  */

  //5->6) After you're done with that, now go back to end.txt
}
