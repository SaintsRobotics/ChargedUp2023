// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);

  /** These limit switches go LOW when they detect something */
  private final DigitalInput m_minLimit = new DigitalInput(ArmConstants.kMinLimitPort);
  /** These limit switches go LOW when they detect something */
  private final DigitalInput m_maxLimit = new DigitalInput(ArmConstants.kMaxLimitPort);

  // PID for correcting arm angle (both when changing pivot and to counteract
  // gravity)
  private final PIDController m_pivotPID = new PIDController(0.03, 0, 0.002); // TODO: tune pivot PID controller
  /** setpoint is in meters */
  public final PIDController m_elevatorPID = new PIDController(1.5, 0, 0);

  public boolean seenSwitch = false; // Elevator won't move down until we see the switch

  private double m_encoderElevatorOffset = -0.149678698451591; // Encoder offset, updtated whenever a limit switch goes LOW, in meters
  
  // Keeps track of whether we want to allow pivot to raise. This is used to stop the terrible oscilation when the pivot fully retracts (because the spring keeps pushing the arm away from the setpoint
  private boolean plock = false;

  //Keeps track if we want to stop PIDs
  private boolean stopPID = false;

  /**
   * Constructs a {@link ArmSubsystem}.
   * Does not move arm above limit switch automatically and always moves arm to
   * resting position
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-180, 180);
    m_pivotEncoder.configMagnetOffset(ArmConstants.kPivotEncoderOffset);
    m_pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    
    m_pivotPID.setTolerance(10); //NOTE: this line may be a reason for PID issues in future code

    m_elevatorMotor.setInverted(true);
  }

  /**
   * Runs on robot enable and resets PID controller and sets seenSwitch to false
   */
  public void enable() {
    m_pivotPID.reset();
    m_pivotPID.setSetpoint(m_pivotEncoder.getAbsolutePosition());

    m_elevatorPID.reset();
    m_elevatorPID.setSetpoint(getElevatorEncoder());

    m_elevatorMotor.set(0);
    m_pivotMotor.set(0);

    seenSwitch = false;
    stopPID = false;
  }

  @Override
  public void periodic() {
    double ps = m_pivotPID.calculate(m_pivotEncoder.getAbsolutePosition());
    double es = m_elevatorPID.calculate(getElevatorEncoder());

    SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder() * 39.3); // inches
    // SmartDashboard.putNumber("elevator distance difference (length until max)", getElevatorMax() - getElevatorEncoder() * 39.3);
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("elevator max extension", getElevatorMaxCheckpoints() * 39.3);
    SmartDashboard.putNumber("theoretical distance past frame", 39.3 * (getElevatorEncoder() * Math.sin(Math.toRadians(m_pivotEncoder.getAbsolutePosition())) - ArmConstants.kAxleToFrontPerimeter));
    SmartDashboard.putNumber("theoretical height", 39.3 * (getElevatorEncoder() * Math.cos(Math.toRadians(m_pivotEncoder.getAbsolutePosition())) + ArmConstants.kPivotAxleHeight));

    /*
    //TODO: remove local variable and some smart dashboard outputs
    
    SmartDashboard.putNumber("Elevator Output Current (Amps)", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Output Current (Amps)", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator Relative Encoder m", getElevatorEncoder()); // this encoder has a conversion
                                                                                   // factor method. We want non
                                                                                   // absolute encoder because values
                                                                                   // can go past 360 and we need to
                                                                                   // know that
    SmartDashboard.putBoolean("Min Limit Switch", !m_minLimit.get());
    SmartDashboard.putBoolean("Max Limit Switch", !m_maxLimit.get());
    SmartDashboard.putBoolean("Seen Switch", seenSwitch);
    SmartDashboard.putNumber("elevator speed", m_elevatorMotor.get());
    SmartDashboard.putNumber("Elevator PID error", m_elevatorPID.getPositionError());
    SmartDashboard.putNumber("Elevator PID setpoint", m_elevatorPID.getSetpoint());
    SmartDashboard.putNumber("Pivot Amps", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot PID out |", ps);
    SmartDashboard.putNumber("Pivot PID in |", m_pivotPID.getSetpoint());
    SmartDashboard.putNumber("Pivot Error", m_pivotPID.getPositionError());
    SmartDashboard.putNumber("Elevator Out", es);
    SmartDashboard.putBoolean("Sp", stopPID); 
    */

    //TODO: drop current limit to 25Amps
    //Set seenSwitch to true if we see a switch
    if (!m_minLimit.get() || !m_maxLimit.get()) {
      seenSwitch = true;
    }

    //Correct encoder and setpoint when we hit bottom limit
    if (!m_minLimit.get()) {
      m_encoderElevatorOffset = ArmConstants.kMinSwitchPos - encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
      m_pivotPID.setSetpoint(m_pivotEncoder.getAbsolutePosition());
      SmartDashboard.putString("return", "minlimit");
      return;
    }

    //Correct encoder when we hit top limit
    if (!m_maxLimit.get()) {
      m_encoderElevatorOffset = ArmConstants.kMaxSwitchPos - encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
      SmartDashboard.putString("return", "maxlimit");
      return;
    }

    // if (seenSwitch) { // TODO: TEST instead of the 2 if statements below
    //   m_elevatorPID.setSetpoint(MathUtil.clamp(m_elevatorPID.getSetpoint(), ArmConstants.kMinSwitchPos + 0.075, ArmConstants.kMaxSwitchPos - 0.01));
    // }

    //Check for underextension (physical), if so increase the setpoint
    if (m_elevatorPID.getSetpoint() < (ArmConstants.kMinSwitchPos + 0.075) && seenSwitch) {
      m_elevatorPID.setSetpoint(ArmConstants.kMinSwitchPos + 0.075);
      SmartDashboard.putString("return", "under");
      return;
    }
    
    
    //Check for overextentsion, of so decrease the setpoint
    if (m_elevatorPID.getSetpoint() > ArmConstants.kMaxSwitchPos - 0.01) {
      m_elevatorPID.setSetpoint(ArmConstants.kMaxSwitchPos - 0.01);
      SmartDashboard.putString("return", "overext");
      return;
    }

    //Don't allow retraction before seeing limit switch
    if (m_elevatorPID.getSetpoint() < getElevatorEncoder() && !seenSwitch) {
      m_elevatorPID.setSetpoint(getElevatorEncoder());
      SmartDashboard.putString("return", "limitR");
      return;
    }

    // SmartDashboard.putNumber("elevator distance difference (length until max)", (getElevatorMaxCheckpoints() - getElevatorEncoder()) * 39.3);
    // SmartDashboard.putNumber("elevator", getElevatorEncoder());
    // SmartDashboard.putNumber("max ext", getElevatorMaxCheckpoints());
    // SmartDashboard.putBoolean("elevatorMaxCheck", m_elevatorPID.getSetpoint() > getElevatorMaxCheckpoints());
    // if (m_elevatorPID.getSetpoint() > getElevatorMaxCheckpoints()) {
    //   m_elevatorMotor.set(0);
    //   m_elevatorPID.setSetpoint(getElevatorEncoder());
    //   SmartDashboard.putNumber("elevatorMaxSetpoint", m_elevatorPID.getSetpoint());

    //   return;
    // }

    //Stop pivoting backwards
    if (m_pivotPID.getSetpoint() < ArmConstants.kMinPivotPos + 3) {
      m_pivotMotor.set(0);
      
      //Stops oscilation (due to spring and low gravitational influence on torque)
      plock = true;
      SmartDashboard.putString("return", "bkl");
      return;
    }
    
    //Allows more pivot movement if we are not at risk of oscilation due to the spring
    plock = false;

    //Stop pivoting too low
    if (m_pivotPID.getSetpoint() > ArmConstants.kMaxPivotPos - 3) {
      m_pivotPID.setSetpoint(ArmConstants.kMaxPivotPos - 3);
      SmartDashboard.putString("return", "tooLo");
      return;
    }

    SmartDashboard.putString("return", "none");

    //Only pivot if we won't hit the pivot motor mounting area
    //TODO: add anti oscilation with POV setpoints (if any setpoint may cause oscilation, current anti-oscilation code via plock only works with manual control)
    if (seenSwitch || ps < 0 || m_pivotPID.getSetpoint() <= 20) {
      if (!stopPID)
        m_pivotMotor.set(MathUtil.clamp(ps, -0.3, 0.3));
    }
  
    if (!stopPID) {
      double motorSpeed = MathUtil.clamp(es, -0.25, 0.25)
      + (0.015 * Math.cos(m_pivotEncoder.getAbsolutePosition()));
      m_elevatorMotor.set(motorSpeed);
      SmartDashboard.putNumber("motorSpeed", motorSpeed);
    } else {
      SmartDashboard.putNumber("motorSpeed", 999);
    }

  }

  /**
   * Toggles whether or not we use PIDs and resets PIDs ands setpoints and stops motor
   */
  public void togglePID(){
    m_elevatorMotor.set(0);
    m_pivotMotor.set(0);

    m_pivotPID.reset();
    m_pivotPID.setSetpoint(m_pivotEncoder.getAbsolutePosition());

    m_elevatorPID.reset();
    m_elevatorPID.setSetpoint(getElevatorEncoder());

    stopPID = !stopPID;
  }

  /**
   * Makes the arm go into resting position
   */
  public void goResting() {
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
  }

  /**
   * Makes the arm go into top position
   */
  public void goTop() {
    m_pivotPID.setSetpoint(ArmConstants.kTopPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kTopPos);
  }

  /**
   * Makes the arm go into mid position
   */
  public void goMid() {
    m_pivotPID.setSetpoint(ArmConstants.kMidPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kMidPos);
  }

  /**
   * Makes the arm go into station position
   */
  public void goStation() {
    m_pivotPID.setSetpoint(ArmConstants.kStationPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kStationPos);
  }

  /**
   * Sets the speeds of the arm elevator and pivot motors.
   *
   * @param pivotSpeed    The desired speed of the pivot motor.
   * @param elevatorSpeed The desired speed of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {
    //Don't adjust arm speed if no input, technically uneeded in this implementation but leave it just to be safe
    if (pivotSpeed == 0 && elevatorSpeed == 0) {
      return;
    }
    
    //Only allow input if it won't cause oscilations or pivoting too far back
    if (!plock || pivotSpeed > 0) {
      m_pivotPID.setSetpoint(m_pivotPID.getSetpoint() + (pivotSpeed * Robot.kDefaultPeriod));
    }
    
    m_elevatorPID.setSetpoint(m_elevatorPID.getSetpoint() + (elevatorSpeed * Robot.kDefaultPeriod));
  }

  /**
   * Converts elevator spark max encoder value to meters
   * 
   * @param encoderValue Number of rotations motor has made
   * @return Current elevator position in meters
   */
  private double encoderToMeters(double encoderValue) {
    // 4:1 gear ratio
    // 22 tooth sproket
    // 1/4 inch chain
    // 39.37 inches in a meter
    // rotation * 4 = 22 teeth * 1/4 inch = 5.5 inches per 4 rotations = 1.375
    // inches per rotation
    // meters = rotation * 1.375 / 39.37
    return encoderValue * (ArmConstants.sproketTeeth * ArmConstants.inchPerTeeth / ArmConstants.gearRatio) / 39.37;
  }

  /**
   * Gets the elevator encoder value in meters with the offset.
   * 
   * @return The elevator encoder value.
   */
  private double getElevatorEncoder() {
    return m_encoderElevatorOffset + encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
    // should setPositionConversionFactor on the motor rather than using
    // encoderToMeters
  }

  // private double getElevatorMaxCheckpoints () {
  //   double pivotEncoderVal = MathUtil.clamp(m_pivotEncoder.getAbsolutePosition(), 0, 90);
  //   if (pivotEncoderVal >= 0 && pivotEncoderVal <= ArmConstants.kTopPivot) {
  //     return 0;
  //   }
  //   if (pivotEncoderVal <= ArmConstants.kMidPivot)
  // }

  private double getElevatorMaxCheckpoints () {
    double pivotEncoderVal = Math.toRadians(MathUtil.clamp(m_pivotEncoder.getAbsolutePosition(), 0, 90));
    
    double max_extension = (ArmConstants.kMaxFrameExtensionLimit + ArmConstants.kAxleToFrontPerimeter) / (pivotEncoderVal == 0 ? 1.0e-10 : Math.sin(pivotEncoderVal));
    max_extension -= ArmConstants.detector2tip + ArmConstants.kLimitToAxle;

    double max_height = (ArmConstants.kMaxExtensionHeight) / (pivotEncoderVal == Math.PI/2 ? 1.0e-10 : Math.cos(pivotEncoderVal));
    max_height -= ArmConstants.floor2ArmBase + ArmConstants.detector2tip;

    return Math.min(max_height, max_extension);
  }

  private double getElevatorMax() {
    double maxHeight = 999;
    double maxExtension = 999;
    double pivotEncoderValue = MathUtil.clamp(m_pivotEncoder.getAbsolutePosition(), 0, 90); // Ensures that the pivot encoder does not update between calls

    // grabberToDetector = value between the limit switch detector and the tip of the grabber
    // extensionLimit = horizontal distance between limit switch and the front of frame

    
    
    if (pivotEncoderValue != 0) {
      maxExtension = (ArmConstants.kMaxFrameExtensionLimit + ArmConstants.kAxleToFrontPerimeter)/Math.sin(Math.toRadians(pivotEncoderValue));
    }
    if (pivotEncoderValue != 90) {
      maxHeight = (ArmConstants.kMaxExtensionHeight - ArmConstants.kPivotAxleHeight)/Math.cos(Math.toRadians(pivotEncoderValue));
    }

    SmartDashboard.putNumber("maxHeight", maxHeight * 39.3);
    SmartDashboard.putNumber("maxExtension", maxExtension * 39.3);
  
    return Math.min(maxHeight, maxExtension) - ArmConstants.kElevatorMaxExtensionOffset; // TODO: Check math with Greg
  }
}

