// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotShaftEncoderPort);

  private final DigitalInput m_restingPosLimit = new DigitalInput(ArmConstants.kRestingLimitPort);
  private final DigitalInput m_topPosLimit = new DigitalInput(ArmConstants.kTopLimitPort);
  private final DigitalInput m_midPosLimit = new DigitalInput(ArmConstants.kMidLimitPort);
  private final DigitalInput m_stationPosLimit = new DigitalInput(ArmConstants.kStationLimitPort);

  private Boolean useLimitSwitch = true; //If the operator uses a non zero manual elevator speed then stop the PID
  private int targetEH = ArmConstants.kRestingEH;
  
  private final PIDController m_pivotPID = new PIDController(0, 0, 0); //TODO: tune this

  /**
   * Constructs a {@link ArmSubsystem}.
   *
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-Math.PI, Math.PI);

    m_pivotEncoder.configFeedbackCoefficient(Math.toRadians(0.087890625), "radians", SensorTimeBase.PerSecond);
    m_pivotEncoder.configMagnetOffset(ArmConstants.kEncoderOffset);
  }

  @Override
  public void periodic() {
    m_pivotMotor.set(m_pivotPID.calculate(m_pivotEncoder.getAbsolutePosition()) * ArmConstants.kMaxPivotSpeedPercent);
    
    
    if (useLimitSwitch){
      if (getEH() >= targetEH) //Higher then desired target
        m_elevatorMotor.set(-ArmConstants.kArmDriveSpeedPercent);

      else //Lowe than target
        m_elevatorMotor.set(ArmConstants.kArmDriveSpeedPercent);
    }
  }

  /**
   * Get the status of all the limit switches in a hash form
   * The hash form has 1 bit for every limit switch with the first being resting, second being mid, third being top,
   * fourth being station. The first bit has a place value of 8 and the final has a value of 1
   * @return The Hash form of the switches' state
   */
  private int getEH(){
    return 
      (m_restingPosLimit.get() ? 0b1000 : 0) + 
      (m_midPosLimit.get() ? 0b0100 : 0) +
      (m_topPosLimit.get() ? 0b0010 : 0) +
      (m_stationPosLimit.get() ? 0b0001 : 0);
  }

  /**
   * Makes the arm go into resting position
   */
  public void goResting(){
    useLimitSwitch = true;
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);
    targetEH = ArmConstants.kRestingEH;
  }

  /**
   * Makes the arm go into top position
   */
  public void goTop(){
    useLimitSwitch = true;
    m_pivotPID.setSetpoint(ArmConstants.kTopPivot);
    targetEH = ArmConstants.kTopEH;
  }

  /**
   * Makes the arm go into mid position
   */
  public void goMid(){
    useLimitSwitch = true;
    m_pivotPID.setSetpoint(ArmConstants.kMidPivot);
    targetEH = ArmConstants.kMidEH;
  }

  /**
   * Makes the arm go into station position
   */
  public void goStation(){
    useLimitSwitch = true;
    m_pivotPID.setSetpoint(ArmConstants.kStationPivot);
    targetEH = ArmConstants.kStationEH;
  }

  /**
   * Sets the speeds of the arm elevator and pivot motors.
   *
   * @param pivotSpeed The desired speed (in percent) of the pivot motor.
   * @param elevatorSpeed The desired speed (in percent) of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {

    //Don't adjust arm if no input (use limit switch instead)
    if (pivotSpeed == 0 && elevatorSpeed == 0) return;

    useLimitSwitch = false;
    m_pivotMotor.set(pivotSpeed);
    m_elevatorMotor.set(elevatorSpeed);

    if (pivotSpeed == 0) m_pivotPID.setSetpoint(m_pivotEncoder.getAbsolutePosition());
  }
}
