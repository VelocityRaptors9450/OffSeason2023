// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class LimelightTurretSubsystem extends SubsystemBase {
  double x;
  double y;
  double area;
  double id;
  double hasTarget;



  GenericEntry limelightx = Shuffleboard.getTab("SmartDashboard")
  .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
  .add("LimelightX", x)
  .withWidget(BuiltInWidgets.kEncoder)
  .withSize(2, 2) //2x2 size
  .getEntry(); 

  GenericEntry limelighty = Shuffleboard.getTab("SmartDashboard")
  .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
  .add("LimelightY", y)
  .withWidget(BuiltInWidgets.kEncoder)
  .withSize(2, 2) //2x2 size
  .getEntry(); 
  
  GenericEntry limelightarea = Shuffleboard.getTab("SmartDashboard")
  .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
  .add("Limelight Area", area)
  .withWidget(BuiltInWidgets.kEncoder)
  .withSize(2, 2) //2x2 size
  .getEntry(); 
  
  GenericEntry tagid = Shuffleboard.getTab("SmartDashboard")
  .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
  .add("Tag ID", id)
  .withWidget(BuiltInWidgets.kEncoder)
  .withSize(2, 2) //2x2 size
  .getEntry(); 
  
  GenericEntry hastarget = Shuffleboard.getTab("SmartDashboard")
  .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
  .add("Has Target", hasTarget)
  .withWidget(BuiltInWidgets.kEncoder)
  .withSize(2, 2) //2x2 size
  .getEntry(); 

  GenericEntry absencoder;
  
  
    




  

  // PID AND FEEDFORWARD TO BE TUNED
  CANSparkMax turret = new CANSparkMax(Constants.turretId, MotorType.kBrushless); // 4 : 3 : 14 : 1 (gear ratios)
  private final PIDController turretPIDController =
    new PIDController(0.02, 0, 0);
  public SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0, 0, 0); 


  // if absolute encoder plugged into cansparkmax:
  SparkMaxAbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder(Type.kDutyCycle);

  
  // obtaining data with network tables for limelight; see here: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api 
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) (prob bigger range b/c LL3)
  NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) (prob bigger range b/c LL3)
  NetworkTableEntry ta = table.getEntry("ta"); // 	Target Area (0% of image to 100% of image)
  NetworkTableEntry tagID = table.getEntry("tid"); // 	id of primary tag in view
  NetworkTableEntry tv = table.getEntry("tv"); // returns either 0 or 1 if there is any tag in frame  
  //returns vision derived pose
  double[] visionPose = table.getEntry("botpose").getDoubleArray(new double[6]); // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
  
  
  public LimelightTurretSubsystem() {
    turret.setIdleMode(IdleMode.kBrake);

    // turretThings.add("LimelightX", x);
    // turretThings.add("LimelightY", y);
    // turretThings.add("LimelightArea", area);
    // turretThings.add("Tag ID", id);
    // turretThings.add("Has Target", hasTarget);
    // turretThings.add("Turret Abs Encoder", turretEncoder.getPosition());
    

    absencoder = Shuffleboard.getTab("SmartDashboard")
    .getLayout("Turret_Limelight", BuiltInLayouts.kGrid)
    .add("Turret Abs Encoder", turretEncoder.getPosition())
    .withWidget(BuiltInWidgets.kEncoder)
    .withSize(2, 2) //2x2 size
    .getEntry(); 
    
  }

  public void controllerRotate(DoubleSupplier strafe) {
    double percentOutput = MathUtil.clamp(strafe.getAsDouble(), -0.5, 0.5);
    double voltage = convertToVolts(percentOutput);
    
    if (strafe.getAsDouble() == 0) {
      trackAprilTag(hasTarget);
    } else if (getTurretPosAngle() <= Constants.maxTurretPosition && getTurretPosAngle() >= Constants.minTurretPosition) {
      turret.setVoltage(voltage);
    } else if (getTurretPosAngle() > Constants.maxTurretPosition) {
      turret.stopMotor();
      // only able to move opposite direction
      if (strafe.getAsDouble() > 0) { // 
        turret.setVoltage(voltage);
      }
    } else if (getTurretPosAngle() < Constants.minTurretPosition) {
      turret.stopMotor();
      // only able to move opposite direction
      if (strafe.getAsDouble() < 0) {
        turret.setVoltage(voltage);
      }
    }
  }

  public void shootNoMatterPosition() {
    // max distance:
    double maxDistance = 0;
    // min distance:
    double minDistance = 0;
    if (getDistance() < maxDistance || getDistance() > minDistance) { //if between the max and min values
      if (Math.abs(x) > 0) {
        // turn turret the opposite value of x --> Math.signum(x)
        // in this case, turning turret left increases encoder pos, so +x below
        updateTurretAngle(getTurretPosAngle() + x, hasTarget);
      } 
    } else {
      // don't turn turret
    }

  }

  public void trackAprilTag(double hasTarget) {
    if (Math.abs(x) > 0) {
      // turn turret the opposite value of x --> Math.signum(x)
      // in this case, turning turret left increases encoder pos, so +x below
    
        
        updateTurretAngle(getTurretPosAngle() - x, hasTarget);
        
      
    } else {
      turret.stopMotor();
    }
  }


  // range from .1 to .9 (absolute encoder)
  double oldTurretVel = 0;
  double oldTime = 0;
  double oldAngle = 0;
  public void updateTurretAngle(double goalAngle, double hasTarget) {
    double pidValue = turretPIDController.calculate(getTurretPosAngle(), goalAngle);
    double changeInTime = Timer.getFPGATimestamp() - oldTime;
    double velSetpoint = getTurretPosAngle() - oldAngle / changeInTime;
    double accel = (velSetpoint - oldTurretVel) / (changeInTime); 
    double ffVal = turretFeedForward.calculate(velSetpoint, accel); //takes velocity, and acceleration
    
    double percentOutput = MathUtil.clamp(pidValue + ffVal, -1.0, 1.0);
    double voltage = convertToVolts(percentOutput);
    
    SmartDashboard.putNumber("PID Value", pidValue);
    SmartDashboard.putNumber("Feed Forward", ffVal);
    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("Position error", turretPIDController.getPositionError());
    SmartDashboard.putBoolean("broooooooo", hasTarget == 0.0);
    


    

    if (Math.abs(getTurretPosAngle() - goalAngle) <= 0.3 /*degrees*/) { // no voltage
      SmartDashboard.putBoolean("Is Centered", true);

      turret.setVoltage(0);
    } else { // set voltage
      SmartDashboard.putBoolean("Is Centered", false);
      if (getTurretPosAngle() <= Constants.maxTurretPosition && getTurretPosAngle() >= Constants.minTurretPosition) {
        turret.setVoltage(-voltage);
      } else if (getTurretPosAngle() > Constants.maxTurretPosition) {
        turret.stopMotor();
        // only able to move opposite direction
        if (x > 0) {
          turret.setVoltage(-voltage);
        }
      } else if (getTurretPosAngle() < Constants.minTurretPosition) {
        turret.stopMotor();
        // only able to move opposite direction
        if (x < 0) {
          turret.setVoltage(-voltage);
        }
      }
      
      
     
      
    }
    
    
    // update vars for determining acceleration later
    oldTurretVel =  velSetpoint; 
    oldTime = Timer.getFPGATimestamp(); 
    oldAngle = getTurretPosAngle();
    
  }

  

  private double convertToVolts(double percentOutput){
    return percentOutput * Robot.getInstance().getVoltage();
}

  public double getTurretPosAngle() {
    // double turretGearRevolutions = turret.getEncoder().getPosition() / 9 / 14; // 9:1   14:1 (gear ratios)
    // return turretGearRevolutions * 360;  
    return turretEncoder.getPosition() * 360;
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

  public double[] getVisionPose() {
    return visionPose;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id = tagID.getDouble(0.0);
    hasTarget = tv.getDouble(0.0);
    visionPose = table.getEntry("botpose").getDoubleArray(new double[6]);


    //shootNoMatterPosition();
   
    //updateTurretAngle(.3 * 360);
    //turret.set(0.1);
    
    //post to smart dashboard periodically
    /*
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Turret Abs Encoder", turretEncoder.getPosition());
    SmartDashboard.putNumber("ID", id);
    SmartDashboard.putNumber("has Target", hasTarget);
    */
  
    // System.out.print(hasTarget);
    // System.out.println(hasTarget == 0);
    // System.out.println(id);
    // trackAprilTag(hasTarget);
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
