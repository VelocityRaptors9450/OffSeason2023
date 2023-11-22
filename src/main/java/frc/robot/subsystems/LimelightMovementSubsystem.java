// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LimelightMovementSubsystem extends SubsystemBase {
  double x;
  double y;
  double area;

  CANSparkMax turret = new CANSparkMax(Constants.turretId, MotorType.kBrushless);
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  public LimelightMovementSubsystem() {}


  public void shootNoMatterPosition() {
    // max distance:
    double maxDistance = 0;
    // min distance:
    double minDistance = 0;
    if (getDistance() < maxDistance || getDistance() > minDistance) { //if between the max and min values
      // change angle/voltage etc.
      
    }

  }

  public double getDistance() {
    double targetOffsetAngle_Vertical = y;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 

    // distance from the target to the floor
    double goalHeightInches = 60.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
  /*
      tv Whether the limelight has any valid targets (0 or 1)
      tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
      ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
      ta Target Area (0% of image to 100% of image) 
   * 
   * https://readthedocs.org/projects/limelight/downloads/pdf/latest/
   */

}
