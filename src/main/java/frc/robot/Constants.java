package frc.robot;

public final class Constants
{  
  public static class OperatorConstants
  {
      // Controller Port
      public static final int DRIVER_CONTROLLER_PORT = 0;

      /* DriveTrain Motors */
      public static final int frontLeft = 1;
      public static final int frontRight = 2;

      public static final int backLeft = 3;
      public static final int backRight = 4;

      /* Shooter Motors */
      public static final int leftSpin = 5;
      public static final int rightSpin = 6;
  }


  public static class NumericConstants
  {
    // Controller Joystick Deadzone
    public static final double deadzone = 0.05;

    /* Motor Constants */
    public static final double shooterSpeed = 0.2;
    public static final double driveSpeedLimit = 0.5;
    public static final double turnSpeedLimit = 0.3;
  }
}
