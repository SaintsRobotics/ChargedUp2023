// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** Drives the arm to a desired position. */
public class ArmCommand extends CommandBase {
  private final ArmSubsystem m_subsystem;

  private final ProfiledPIDController m_pivotPID = new ProfiledPIDController(0.03, 0, 0.002, new Constraints(1, 1));
  private final PIDController m_elevatorPID = new PIDController(1.5, 0, 0);

  /**
   * Creates a new {@link ArmCommand}.
   * 
   * @param subsystem        The required subsystem.
   * @param pivotSetpoint    The setpoint for the pivot.
   * @param elevatorSetpoint The setpoint for the elevator.
   */
  public ArmCommand(ArmSubsystem subsystem, double pivotSetpoint, double elevatorSetpoint) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_pivotPID.setGoal(pivotSetpoint);
    m_elevatorPID.setSetpoint(elevatorSetpoint);

    m_pivotPID.setTolerance(10);
    m_elevatorPID.setTolerance(1);

    m_pivotPID.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    m_subsystem.set(
        m_pivotPID.calculate(m_subsystem.getPivotPosition()),
        m_elevatorPID.calculate(m_subsystem.getElevatorPosition()));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.set(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_pivotPID.atSetpoint() && m_elevatorPID.atSetpoint();
  }
}
