// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathWeaverCommand;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                inputScaling(MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                inputScaling(MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                inputScaling(MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                !m_driverController.getRightBumper()),
            m_robotDrive));

    m_chooser.addOption("BlueBottomTwoObject", "BlueBottomTwoObject");
    m_chooser.addOption("BlueTopTwoObject", "BlueTopTwoObject");
    m_chooser.addOption("BlueMidBackCharger", "BlueMidBackCharger");
    m_chooser.addOption("BlueMidFrontCharger", "BlueMidFrontCharger");
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String path;
    if (m_chooser.getSelected() != null) {
      path = m_chooser.getSelected();
    } else {
      return null;
    }

    switch (path) {
      case ("BlueBottomTwoObject"):
        return new SequentialCommandGroup(
            new PathWeaverCommand(m_robotDrive, path + "1", true),
            new PathWeaverCommand(m_robotDrive, path + "2", false));
      case ("BlueMidBackCharger"):
        return new PathWeaverCommand(m_robotDrive, path + "1", true);
      case ("BlueTopTwoObject"):
        return new SequentialCommandGroup(
            new PathWeaverCommand(m_robotDrive, path + "1", true),
            new PathWeaverCommand(m_robotDrive, path + "2", false));
      case ("BlueMidFrontCharger"):
        return new SequentialCommandGroup(
            new PathWeaverCommand(m_robotDrive, path + "1", true),
            new PathWeaverCommand(m_robotDrive, path + "2", false));
      default:
        return null;
    }
  }

  /**
   * Makes lower inputs smaller which allows for finer joystick control.
   * 
   * @param input The number to apply input scaling to.
   * @return The scaled number.
   */
  private double inputScaling(double input) {
    return input * Math.abs(input);
  }
}
