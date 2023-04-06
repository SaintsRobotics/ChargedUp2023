// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignCommand extends CommandBase {

  /**
   * Direction to align to
   */
  public enum Direction {
    kLeft, kRight, kCenter
  }

  /** Drive subsystem */
  private final DriveSubsystem m_subsystem;

  /** Direction to align to */
  private final Direction m_direction;

  /** Is command finished or is robot is bad initial state (ex: too far away) */
  private boolean m_isFinished;

  /** PID for robot Y speed */
  private final PIDController m_PID = new PIDController(0, 0, 0); // TODO: tune this

  /**
   * Keeps track of how long we are at the setpoint to check if the command should
   * end
   */
  private final Timer m_atSetpointTimer = new Timer();

  /** Keeps track if we were at setpoint to determine if command should end */
  private boolean m_wasAtSetpoint;

  /** Creates a new AutoAlign. */
  public AutoAlignCommand(DriveSubsystem subsystem, Direction direction) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    // Check if we need to invert direction because of alliance side
    if (DriverStation.getAlliance() == Alliance.Red) {
      switch (direction) {
        case kLeft:
          m_direction = Direction.kRight;
          break;
        case kRight:
          m_direction = Direction.kLeft;
          break;
        default:
          m_direction = Direction.kCenter;
      }
    }
    else {
      m_direction = direction;
    }

    m_PID.setTolerance(AlignConstants.kPIDTolerance);
  }

  @Override
  public void initialize() {
    int selection = 0; // Closest starting position
    m_isFinished = m_wasAtSetpoint = false;

    double positionY = m_subsystem.getPose().getY();
    double positionX = m_subsystem.getPose().getX();

    // Get starting position
    for (int i = 1; i < 11; i++) {
      if (Math.abs(positionY - AlignConstants.kAlignPositions[i]) < Math
          .abs(positionY - AlignConstants.kAlignPositions[selection])) {
        selection = i;
      }
    }

    // Check if we are already too far to the left
    if (selection == 10 && m_direction == Direction.kLeft) {
      m_isFinished = true;
    }

    // Check if we are too far to the right
    else if (selection == 0 && m_direction == Direction.kRight) {
      m_isFinished = true;
    }

    // Check if we are too far from grid
    else if (selection < 9 && (positionX < AlignConstants.kXPositionGrid - AlignConstants.kXPositionToleranceMeters
        || positionX > AlignConstants.kXPositionGrid + AlignConstants.kXPositionToleranceMeters)) {
      m_isFinished = true;
    }

    // Check if we are too far from the loading
    else if (selection < 9 && (positionX < AlignConstants.kXPositionLoad - AlignConstants.kXPositionToleranceMeters
        || positionX > AlignConstants.kXPositionLoad + AlignConstants.kXPositionToleranceMeters)) {
      m_isFinished = true;
    }

    // Change selection for left direction
    if (m_direction == Direction.kLeft)
      selection++;

    // Change selection for right direction
    else if (m_direction == Direction.kRight)
      selection--;

    m_PID.setSetpoint(AlignConstants.kAlignPositions[selection]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Don't do anything if we are finished
    if (m_isFinished) {
      return;
    }

    // Check if we are aligned
    if (m_PID.atSetpoint()) {

      // Stop driving
      m_subsystem.drive(0, 0, 0, true);

      // Check if we just got aligned
      if (!m_wasAtSetpoint) {
        m_wasAtSetpoint = true;
        m_atSetpointTimer.restart();

      // Make sure we remain aligned for a certain amount of time before finishing the command
      } else if (m_atSetpointTimer.hasElapsed(AlignConstants.kAtSetpointTimeSeconds)) {
        m_isFinished = true;
      }
    }

    m_wasAtSetpoint = false;

    // Drive the robot to align
    m_subsystem.drive(0, MathUtil.clamp(m_PID.calculate(m_subsystem.getPose().getY()),
        -AlignConstants.kMaxSpeedMetersPerSecond, AlignConstants.kMaxSpeedMetersPerSecond), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
