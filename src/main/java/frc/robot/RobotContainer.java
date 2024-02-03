//3) Now that you here, we are going to learn how to use controllers such as the Xbox controller Now go to the next "3)"comment


package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/*3) This is the line where we will import the library for the 
CommandXboxController that we will use to implement controller inputs*/
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  
  //3) We will also need to re-create a new RomiDrivetrain object in this file, call it drive.
  
  /*3) We can see the RomiDrivetrain is here above as an object called drive
  Below, we have declared an CommandXboxController object called controller for
  Xbox controller inputs.
  We can see this object uses a constructor because it has parameters, this parameter is the port value
  of the controller we use, this value doesn't metter much though for now. After inspecting the line below, 
  go to the bottom of the file and search for the the next "3)" file*/
  public CommandXboxController controller = new CommandXboxController (0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */



  /*  3) Now lets work to create 2 simple methods that will run the motors according to a controller joystick. 
  The first method will use 1 joystick and the second will use 2 joysticks*/
  /*first go to https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html 
  to see controller documentation

  The first void method will be called goWithController1();
  Use these to methods to get the xAxis and yAxis position of the left joystick. 
  controller.getLeftY(); for yAxis
  controller.getLeftx(); for xAxis

  reveiw these methods if needed using the link

  use the drive.go(left,right); to get the motors to move at the right speeds 
  Use these equations to get the right moter values:
  left = ( (-yAxis / 2) - ( xAxis / 2 ) ) * 2
  right = ( ( -yAxis / 2 ) + ( xAxis / 2 ) ) * 2

  now write the method below
  */




  /*Now that you're done with that method lets make another method called goWithController2();
  Use these to methods to get the rightValue and leftValue position of the left joystick. 
  controller.getLeftY(); for left
  controller.getRightY(); for right

  reveiw these methods if needed using the link

  use the drive.go(left,right); to get the motors to move at the right speeds 
  You shoudldn't any equations for this one just apply each joystick value to their corresponding wheel
  
  Now write the method below
   */
  



  
  /*Now that you're done with the methods, go to the Robot.java and create a new RobotContainer object called container. Then go to the
  teleopPeriodic() method and test your methods by calling the methods above from this class in the teleopPeriodic() method.
  *Remeber to do one method at a time to avoid problems.
  Rememeber to come back to this comment afterwards (look down to the next comment for the next intructions)*/

 
  //3->4) Now go back to the Robot.java file and search for the seconds step of this section at the top.
  
}