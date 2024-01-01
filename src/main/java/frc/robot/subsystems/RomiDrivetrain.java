

//1) GO TO THE BOTTOM TO START CODING


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
  
  /* New method  <motor>.set(double power);    --> sets the given motor to the power(double) forever until changed or inturupped.
  PS: the power value is between 0 and 1 (0% to 100%)*/

  // use the moter variables leftMotor and rightMotor to create a void method called   go(double left_power, double right_power)
  //this method shoud apply the left_power parameter as the power of the left motor and the right_power parameter as the power of the right motor.
  //Hint: use the <motor>.set method with the leftMotor and rightMotor vairables for the motors. Write the code below.

  //2) Now we have to get this code to run so go to Robot.java




}

