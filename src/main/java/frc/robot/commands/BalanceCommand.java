// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Queue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LEDEffectCommand.EffectType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/** Uses a PID and the gyroscope to balance the robot on the charger. */
public class BalanceCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController m_PID = new PIDController(0.025, 0, 0);
  private final Timer m_timer = new Timer();
  private final LEDSubsystem m_LEDSubsystem;
  private final Queue<SequentialCommandGroup> m_effectQueue;

  /**
   * Creates a new {@link BalanceCommand}.
   * 
   * @param driveSubsystem The required drive subsystem.
   * @param LEDSubsystem   The required LED subsystem.
   */
  public BalanceCommand(DriveSubsystem driveSubsystem, LEDSubsystem LEDSubsystem,
      Queue<SequentialCommandGroup> effectQueue) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_effectQueue = effectQueue;

    m_LEDSubsystem = LEDSubsystem;
    m_PID.setTolerance(DriveConstants.kToleranceBalance);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    m_driveSubsystem.drive(
        m_PID.calculate(m_driveSubsystem.getGyroPitch(), 0),
        0,
        0,
        false);

    if (m_timer.hasElapsed(1) && m_PID.atSetpoint()) {
      SequentialCommandGroup cmd = new SequentialCommandGroup(
          new LEDEffectCommand(
              m_LEDSubsystem, EffectType.swipeUp, 0, 100, 0, 0.02, () -> {
                return false;
              }),
          new LEDEffectCommand(
              m_LEDSubsystem, EffectType.swipeDown, 0, 100, 0, 0.02, () -> {
                return false;
              }));

      cmd.schedule();
      m_effectQueue.add(cmd);
      m_timer.reset();

    } else if (m_timer.hasElapsed(0.6)) {
      SequentialCommandGroup cmd = new SequentialCommandGroup(new LEDEffectCommand(
          m_LEDSubsystem, EffectType.blink, 100, 75, 0, 0.2, () -> {
            return false;
          }));

      cmd.schedule();
      m_effectQueue.add(cmd);
      m_timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }
}
