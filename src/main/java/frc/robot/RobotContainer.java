// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.DriveSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final BalanceCommand m_BalanceCommand = new BalanceCommand(m_robotDrive);
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_colorChooser = new SendableChooser<>();

  private final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose,
      m_robotDrive::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(DriveConstants.kPTranslation, 0, 0),
      new PIDConstants(DriveConstants.kPRotation, 0, 0),
      m_robotDrive::setModuleStates,
      m_eventMap,
      m_robotDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // Holding left trigger engages slow mode
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    OIConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController.getLeftTriggerAxis() * OIConstants.kSlowModeScalar),
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    OIConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController.getLeftTriggerAxis() * OIConstants.kSlowModeScalar),
                MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    OIConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                !m_driverController.getRightBumper()),
            m_robotDrive));


     m_colorChooser.addOption("BlueBottomTwoObject", "BlueBottomTwoObject"); 
     m_colorChooser.addOption("RedBottomTwoObject", "RedBottomTwoObject");

    m_chooser.addOption("BlueBottomCharger", "BlueBottomCharger");
    m_chooser.addOption("BlueBottomTwoObject", "BlueBottomTwoObject");
    m_chooser.addOption("BlueMidBackCharger", "BlueMidBackCharger");
    m_chooser.addOption("BlueMidFrontCharger", "BlueMidFrontCharger");
    m_chooser.addOption("BlueTopCharger", "BlueTopCharger");
    m_chooser.addOption("BlueTopTwoObject", "BlueTopTwoObject");
    m_chooser.addOption("BlueBottomTwoObjectCharger", "BlueBottomTwoObjectCharger");
    m_chooser.addOption("BlueTopTwoObjectCharger", "BlueTopTwoObjectCharger");
    m_chooser.addOption("BlueTopThreeObject", "BlueTopThreeObject");
    m_chooser.addOption("BlueBottomThreeObject", "BlueBottomThreeObject");
    // Sample event that triggers when BlueBottomCharger is run
    m_eventMap.put("event", new WaitCommand(1));

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
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(m_BalanceCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String color;
    if (m_colorChooser.getSelected() != null) {
      color = m_colorChooser.getSelected();
    } else {
      return null;

    String path;
    if (m_chooser.getSelected() != null) {
      path = m_chooser.getSelected();
    } else {
      return null;
    }

    switch (color) {
      case ("BlueBottomTwoObject"):
        m_robotDrive.resetOdometry(new Pose2d(new Translation2d(2.20, 0.49), new Rotation2d(180)));
      case ("RedBottomTwoObject"):
        m_robotDrive.resetOdometry(new Pose2d(new Translation2d(14.34175, 0.49), new Rotation2d(0)));

    }

    switch (path) {
      case ("BlueBottomCharger"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueBottomTwoObject"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueMidBackCharger"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueMidFrontCharger"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueTopCharger"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueTopTwoObject"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueBottomTwoObjectCharger"):
        return new SequentialCommandGroup(
          m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("BlueBottomTwoObject", new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration))),
          m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("BlueBottomCharger", new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration))));
      case ("BlueTopTwoObjectCharger"):
        return new SequentialCommandGroup(
          m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("BlueTopTwoObject", new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration))),
          m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("BlueTopCharger", new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration))));
      case ("BlueTopThreeObject"):
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      case ("BlueBottomThreeObject"):
      return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration)));
      default:
        return null;
    }
  }
}


