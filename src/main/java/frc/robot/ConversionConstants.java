// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ConversionConstants {
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
