// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PathWeaverCommand;

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
                DriveSubsystem.oddSquare(MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                DriveSubsystem.oddSquare(MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                DriveSubsystem.oddSquare(MathUtil.applyDeadband(
                    -m_driverController.getRightX(),
                    OIConstants.kControllerDeadband))
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                false),
            m_robotDrive));

    m_chooser.addOption("BlueBottemTwoObject", "BlueBottem TwoObject");
    m_chooser.addOption("BlueTopTwoObject", "BlueTop TwoObject");
    m_chooser.addOption("BlueMidBackCharger", "BlueMid BackCharger");
    m_chooser.addOption("BlueMidFrontCharger", "BlueMid FrontCharger");
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String[] path;
    if (m_chooser.getSelected() != null) {
      path = m_chooser.getSelected().split(" ");
    } else {
      return null;
    }

    // Path to drop off loaded object and grab another, straight line, feild
    SmartDashboard.putString("Check in", "1");
    SequentialCommandGroup twoObjectDropOff = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PathWeaverCommand(m_robotDrive, path[0] + "TwoObject1", true)
       
        ),
        new ParallelCommandGroup(
            new PathWeaverCommand(m_robotDrive, path[0] + "TwoObject2", false)
        // System.out.print("Pause to drop off obj")
        ));
    // Drops off object then Path to get on charger from the middle, not leaving
    // the zone
    SequentialCommandGroup GetOnChargerBack = new SequentialCommandGroup(
        new ParallelCommandGroup(
            // Pause to place loaded object
            new PathWeaverCommand(m_robotDrive, path[0] + "BackCharger1", true)

        ));
    // Drops off object, moves over charger to
    SequentialCommandGroup GetOnChargerFront = new SequentialCommandGroup(
        new ParallelCommandGroup(
            // Pause to place loaded object
             new PathWeaverCommand(m_robotDrive, path[0] + "FrontCharger1", true)),

        new ParallelCommandGroup(
            // Pull up arm
            new PathWeaverCommand(m_robotDrive, path[0] + "FrontCharger2", false)));
        SmartDashboard.putString("Check IN", "2");
     switch (path[1]) {
      case ("TwoObject"):
        SmartDashboard.putString("Check in", "3");
        return twoObjectDropOff;
      case ("BackCharger"):
        return GetOnChargerBack;

      case ("FrontCharger"):
        return GetOnChargerFront;
     }
     SmartDashboard.putString("Fail", " ");
     return null;
  }
}
