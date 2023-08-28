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

    public static final double WheelRadius = 0.0508;
    public static final int EncoderResolution = 4096; // tics per revolution
    public static final double DRIVE_MOTOR_CONVERSION = 2 * Math.PI * WheelRadius; // distance 
    public static final double TURNING_MOTOR_CONVERSION = 2 * Math.PI; // distance 
    public static final double VELOCITY_CONVERSION_FACTOR = Math.PI * WheelRadius / 30; //Distance per second
  }

  public static class Speeds{
    public static final double MaxSpeed = 1;
    public static final double MaxAngularSpeed = Math.PI;
  }
  
  public static final int flDriveId = 17; //8
  public static final int flTurnId = 3; //16
  public static final int flAbsoluteId = 0;


  public static final int frDriveId = 6; //9
  public static final int frTurnId = 9; //3 
  public static final int frAbsoluteId = 2;


  public static final int blDriveId = 22; //6
  public static final int blTurnId = 14; //17
  public static final int blAbsoluteId = 4;


  public static final int brDriveId = 16; //14
  public static final int brTurnId = 8; //18
  public static final int brAbsoluteId = 3;

  public static final double baseWidth = 0.4953;
  public static final double baseLength = 0.6477;

  public static final double flAbsoluteEncoderOffset = -0.056757174432278;
  public static final double frAbsoluteEncoderOffset = -3.103236861526966;
  public static final double blAbsoluteEncoderOffset = 3.120110616087914;
  public static final double brAbsoluteEncoderOffset = 3.107838794589043;

  // PID
  public static final double turnKp = 4.0027;
  public static final double turnKd = 0.10234;
  // feed-forward
  public static final double turnKs = 0.2246;
  public static final double turnKv = 0.069612;
  public static final double turnKa = 0.0022882;


  

  //Width:  19.5in 0.4953m
  //Length: 25.5in 0.6477m
}
