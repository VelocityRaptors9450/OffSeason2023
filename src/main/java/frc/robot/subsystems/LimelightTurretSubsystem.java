// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class LimelightTurretSubsystem extends SubsystemBase {
  double x;
  double y;
  double area;

  // PID AND FEEDFORWARD TO BE TUNED
  CANSparkMax turret = new CANSparkMax(Constants.turretId, MotorType.kBrushless); // 4 : 3 : 14 : 1 (gear ratios)
  private final ProfiledPIDController turretPIDController =
    new ProfiledPIDController(0.001,0,0,
    new TrapezoidProfile.Constraints(Constants.Speeds.maxTurretVel, Constants.Speeds.maxTurretAccel));
  public SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0, 0, 0); 


  // if absolute encoder plugged into cansparkmax:
  SparkMaxAbsoluteEncoder turretEncoder = turret.getAbsoluteEncoder(Type.kDutyCycle);

  
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  //returns vision derived pose
  double[] visionPose = table.getEntry("botpose").getDoubleArray(new double[6]);
  
  
  public LimelightTurretSubsystem() {
    turret.setIdleMode(IdleMode.kCoast);
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
        updateTurretAngle(getTurretPosAngle() + x);
      } 
    } else {
      // don't turn turret
    }

  }


  // range from .1 to .9 (absolute encoder)
  double oldTurretVel = 0;
  double oldTime = 0;
  public void updateTurretAngle(double goalAngle) {
    double pidValue = turretPIDController.calculate(getTurretPosAngle(), goalAngle);
    double velSetpoint = turretPIDController.getSetpoint().velocity;
    double accel = (velSetpoint - oldTurretVel) / (Timer.getFPGATimestamp() - oldTime); 
    double ffVal = turretFeedForward.calculate(turretPIDController.getSetpoint().velocity, accel); //takes velocity, and acceleration
    
    double percentOutput = MathUtil.clamp(pidValue + ffVal, -1.0, 1.0);
    double voltage = convertToVolts(percentOutput);
    
    SmartDashboard.putNumber("PID Value", pidValue);
    SmartDashboard.putNumber("Feed Forward", ffVal);
    SmartDashboard.putNumber("Position error", turretPIDController.getPositionError());
    
    
    turret.setVoltage(voltage);
    
    // update vars for determining acceleration later
    oldTurretVel = turretPIDController.getSetpoint().velocity;
    oldTime = Timer.getFPGATimestamp(); 
    
  }

  public void setTurretGoal(double goal) {
    turretPIDController.setGoal(goal);
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
    visionPose = table.getEntry("botpose").getDoubleArray(new double[6]);

    //shootNoMatterPosition();
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Turret Abs Encoder", turretEncoder.getPosition());
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
