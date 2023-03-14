// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax fl = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax fr = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax bl = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax br = new CANSparkMax(2, MotorType.kBrushless);
  WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(39.37));
  //DifferentialDriveOdometry odo = new DifferentialDriveOdometry(kinematics, getHeading());
  public DriveTrain() {
    //passes power to the following motors
    //could run each motor on its own
    fr.follow(fl);
    br.follow(bl);
    //revese motor diections so it won't spin in place
    //left side reversed right side normal
    fl.setInverted(false);
    br.setInverted(true);
    /*
     * When scoring or parking, make wheels a cross x to not move
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
