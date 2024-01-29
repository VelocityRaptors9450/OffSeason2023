//1) GO TO THE BOTTOM AND READ THE COMMENTS THERE TO START CODING


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // These are encoder vsairbles that can be used to see how far are romi has gotten by getting the orientation of the motor
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  //private final DifferentialDrive m_diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);
  }
  /* 
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
  */
  //resets the encoders values
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  //gets how far left motor has turned in inches using encoders
  public double getLeftDistanceInch() {
    return leftEncoder.getDistance();
  }
  //gets how far right motor has turned in inches using encoders
  public double getRightDistanceInch() {
    return rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  //1)Before we start coding, go through the methods and variables above and see what they do.
  
  /*1) New method <motor>.set(double power); --> sets the given motor to the power(double) forever until power is changed or inturupped.
  PS: the power value is between 0 and 1 (0% to 100%) */

  // 1) use the moter variables leftMotor and rightMotor to create a void method called   go(double left_power, double right_power)
  //This method should apply the left_power parameter as the power of the left motor and the right_power parameter as the power of the right motor.
  // Hint: use the <motor>.set method with the leftMotor and rightMotor variables for the motors. Write the code below. After you're done, move to the next step

  
  //1) Now let's quickly learn about encoders
  // Encoders are values that store the number of rotations your motor has made, this can be used to find distance traveled and many other important factors
  /*Now lets create another mathod called go1Foot() that moves the Romi forward 1 foot.
  We can use the already written go method, simply check if the the robot goes 12 inches (only check 1 motor) and if it does, stop.
  Luckily for us, WPILIB created 2 methods called 
  getLeftDistanceInch() that checks how far the left motor went in inches
  getRightDistanceInch() that checks how far the right motor went in inches
  write the code down below*/

  /* 1->2) Now that you're done we can move on to section 2 by going to Robot.java 
  (I would also recommend deleting all comments relating to section 1 for readability)*/




}

