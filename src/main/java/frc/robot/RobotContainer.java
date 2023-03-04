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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.SnapRotateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose,
      m_robotDrive::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(DriveConstants.kPTranslation, 0, 0),
      new PIDConstants(DriveConstants.kPRotation, 0, 0),
      m_robotDrive::setModuleStates,
      m_eventMap,
      true,
      m_robotDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_armSubsystem.setDefaultCommand(
        new RunCommand(() -> m_armSubsystem.set(
            MathUtil.applyDeadband(
                -m_operatorController.getLeftY(),
                OIConstants.kControllerDeadband),
            MathUtil.applyDeadband(
                -m_operatorController.getRightY(),
                OIConstants.kControllerDeadband)),
            m_armSubsystem));

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
                    * (1 - m_driverController.getLeftTriggerAxis() * OIConstants.kSlowModeScalar)
                    / 3,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    OIConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController.getLeftTriggerAxis() * OIConstants.kSlowModeScalar)
                    / 3,
                MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    OIConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    / 3,
                !m_driverController.getRightBumper()),
            m_robotDrive));

    m_chooser.addOption("BottomCharger", "BottomCharger");
    m_chooser.addOption("BottomThreeObject", "BottomThreeObject");
    m_chooser.addOption("BottomTwoObject", "BottomTwoObject");
    m_chooser.addOption("MidBackCharger", "MidBackCharger");
    m_chooser.addOption("MidFrontCharger", "MidFrontCharger");
    m_chooser.addOption("TopCharger", "TopCharger");
    m_chooser.addOption("TopThreeObject", "TopThreeObject");
    m_chooser.addOption("TopTwoObject", "TopTwoObject");
    SmartDashboard.putData(m_chooser);

    m_eventMap.put("BalanceCommand", new BalanceCommand(m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Bindings
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new BalanceCommand(m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new SnapRotateCommand(m_robotDrive));

    // Operator Bindings
    new JoystickButton(m_operatorController, Button.kA.value)
        .onTrue(new InstantCommand(grabberSubsystem::toggle, grabberSubsystem));

    /*
     * DO NOT USE: ROBOT WILL BREAK
     * new POVButton(m_operatorController, 0)
     * .onTrue(
     * new InstantCommand(m_armSubsystem::goStation, m_armSubsystem)); // Up -
     * Station
     * 
     * new POVButton(m_operatorController, 90)
     * .onTrue(
     * new InstantCommand(m_armSubsystem::goTop, m_armSubsystem)); // Left - Top
     * 
     * new POVButton(m_operatorController, 180)
     * .onTrue(
     * new InstantCommand(m_armSubsystem::goResting, m_armSubsystem)); // Down -
     * Resting
     * 
     * new POVButton(m_operatorController, 270)
     * .onTrue(
     * new InstantCommand(m_armSubsystem::goMid, m_armSubsystem)); // Right - Mid
     * 
     */
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

    return m_autoBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            path,
            new PathConstraints(
                AutonConstants.maxVelocity,
                AutonConstants.maxAcceleration)));
  }
}
