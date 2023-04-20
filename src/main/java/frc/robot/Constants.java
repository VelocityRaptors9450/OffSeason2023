// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants{
      public static final int DRIVER_CONTROLLER_PORT = 0;
      public static final int FL_MOVE_MOTOR_ID = 1;
      public static final int FL_TURN_MOTOR_ID = 2;
      public static final int BL_MOVE_MOTOR_ID = 3;
      public static final int BL_TURN_MOTOR_ID = 4;
      public static final int FR_MOVE_MOTOR_ID = 5;
      public static final int FR_TURN_MOTOR_ID = 6;
      public static final int BR_MOVE_MOTOR_ID = 7;
      public static final int BR_TURN_MOTOR_ID = 8;

  }

  public static class ModuleConversion {
    //TOO BE UPDATEEDDDDD
    public static final double WheelDiameter = Units.inchesToMeters(4);
    public static final double DriveMotorGearRatio = 1 / 6;
    public static final double TurnMotorGearRatio = 1 / 6;
    public static final double DriveEncoderRot2Meter = DriveMotorGearRatio * Math.PI * WheelDiameter;
    public static final double TurnEncoderRot2Rad = TurnMotorGearRatio * 2 * Math.PI;
    public static final double DriveEncoderRPM2MeterPerSec = DriveEncoderRot2Meter / 60;
    public static final double TurnEncoderRPM2RadPerSec = TurnEncoderRot2Rad / 60;
    //ticks * 1 rotations/4096 ticks  *  gear ratio  * 6pi inches/1 rotation  * 1 ft / 12 inches
    public static final double drivetcks2ftfactor = 1.0 / 4096 * 6 * Math.PI / 12;

  }
  
  public static final int flDriveId = 16;
  public static final int flTurnId = 15;
  public static final int flAbsoluteId = 0;


  public static final int frDriveId = 6;
  public static final int frTurnId = 8;
  public static final int frAbsoluteId = 0;


  public static final int blDriveId = 17;
  public static final int blTurnId = 3;
  public static final int blAbsoluteId = 0;


  public static final int brDriveId = 14;
  public static final int brTurnId = 9;
  public static final int brAbsoluteId = 0;



}
