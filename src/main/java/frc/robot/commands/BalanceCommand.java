// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Uses a PID and the gyroscope to balance the robot on the charger. */
public class BalanceCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final PIDController m_PID = new PIDController(DriveConstants.kPBalance, 0, 0);

  /**
   * Creates a new {@link BalanceCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public BalanceCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_PID.enableContinuousInput(-180, 180);
    m_PID.setTolerance(DriveConstants.kToleranceBalance);
  }

  @Override
  public void execute() {
    m_subsystem.drive(
        m_PID.calculate(m_subsystem.getGyroPitch(), 0),
        0,
        0,
        true);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return m_PID.atSetpoint();
  }
}
