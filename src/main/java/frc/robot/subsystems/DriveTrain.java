// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax flMove = new CANSparkMax(Constants.OperatorConstants.FL_MOVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax flTurn = new CANSparkMax(Constants.OperatorConstants.FL_TURN_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax blMove = new CANSparkMax(Constants.OperatorConstants.BL_MOVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax blTurn = new CANSparkMax(Constants.OperatorConstants.BL_TURN_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax frMove = new CANSparkMax(Constants.OperatorConstants.FR_MOVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax frTurn = new CANSparkMax(Constants.OperatorConstants.FR_TURN_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax brMove = new CANSparkMax(Constants.OperatorConstants.BR_MOVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax brTurn = new CANSparkMax(Constants.OperatorConstants.BR_TURN_MOTOR_ID, MotorType.kBrushless);


  /* Creates a new DriveTrain. */
  public DriveTrain(){
    flMove.setInverted(true);
    blMove.setInverted(true);
    flTurn.setInverted(true);
    blTurn.setInverted(true);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double forward, double rotate){
    flMove.set(forward);
    frMove.set(forward);
    blMove.set(forward);
    brMove.set(forward);
    flTurn.set(rotate);
    frTurn.set(rotate);
    blTurn.set(rotate);
    brTurn.set(rotate);  
  }

}
