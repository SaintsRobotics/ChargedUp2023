// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase{
  private double m_armSpeed;
  private double m_elevatorSpeed;
  private final CANSparkMax m_armMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);

  private final CANCoder m_armEncoder = new CANCoder(Constants.ModuleConstants.kArmMotorPort);
  private final CANCoder m_elevatorEncoder = new CANCoder(Constants.ModuleConstants.kElevatorMotorPort);


  private final PIDController m_armPIDController = new PIDController(
      ModuleConstants.kPModuleTurningController, 0, 0);


  /**
   * Constructs a {@link ArmSubsystem}.
   *
   * 
   */
  
  public void ArmSubsystem() {

  }

  @Override
  public void periodic() {
    m_armMotor.set(m_armSpeed);
    m_elevatorMotor.set(m_elevatorSpeed);   
    
  }

  public void setArmSpeed(double armSpeed) {
    m_armMotor.set(armSpeed);
  }
  public void setElevatorSpeed(double elevatorSpeed) {
    m_elevatorMotor.set(elevatorSpeed);
  }

}
