// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
  private double m_pivotSpeed;
  private double m_elevatorSpeed;

  private final CANSparkMax m_pivotMotor = new CANSparkMax(Constants.ArmConstants.kPivotMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(Constants.ArmConstants.kElevatorMotorPort, MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(Constants.ArmConstants.kPivotEncoderPort);
  private final CANCoder m_elevatorEncoder = new CANCoder(Constants.ArmConstants.kElevatorEncoderPort);


 


  /**
   * Constructs a {@link ArmSubsystem}.
   *
   * 
   */
  
  public void ArmSubsystem() {

  }

  @Override
  public void periodic() {
    m_pivotMotor.set(m_pivotSpeed);
    m_elevatorMotor.set(m_elevatorSpeed);   
    
  }

  public void setPivotSpeed(double pivotSpeed) {
    m_pivotMotor.set(pivotSpeed);
  }

  public void setElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(elevatorSpeed);
  }

  public double getPivotPosition () {
    return m_pivotEncoder.getPosition();
  }

  public double getElevatorPosition () {
    return m_elevatorEncoder.getPosition();
  }

}
