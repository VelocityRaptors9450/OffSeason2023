// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RomiDrivetrain extends SubsystemBase
{

  // Declares some private constant variables for calculations.
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm


  // Declares the left and right motors.
  private final Spark lSpark = new Spark(0);
  private final Spark rSpark = new Spark(1);


  // Declares the left and right motor encoders (what keeps track of distance travelled).
  private final Encoder lEncoder = new Encoder(4, 5);
  private final Encoder rEncoder = new Encoder(6, 7);


  /* ---------------------------------------------------------------------------- */


  public RomiDrivetrain()
  {
    
    // Uses math to return the distance travelled in inches.
    lEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    rEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);

    // Resets the encoders on initialization.
    resetEncoders();

    // Inverts the right motor since it is flipped.
    rSpark.setInverted(true);

  }


  /* ---------------------------------------------------------------------------- */
  /* All Moving Methods Here */


  /*
   * Moves both motors according to the "x" and "y" values given.
   * The motors are set to the y-value first, and then the
   * "x" values is added or subtracted to allow for turning.
   */
  public void move(double x, double y)
  {
    lSpark.set(y + x);
    rSpark.set(y - x);
  }

  /* ---------------------------------------------------------------------------- */
  /* All Encoder Methods Here */


  /* Resets both encoder values. */
  public void resetEncoders()
  {
    lEncoder.reset();
    rEncoder.reset();
  }


  /* Returns the value given by the left encoder. */
  public double getLeftDistanceInch()
  {
    return lEncoder.getDistance();
  }

  /* Returns the value given by the right encoder. */
  public double getRightDistanceInch()
  {
    return rEncoder.getDistance();
  }


  /* ---------------------------------------------------------------------------- */


  @Override
  public void periodic()
  {

  }

  @Override
  public void simulationPeriodic()
  {
    
  }
}
