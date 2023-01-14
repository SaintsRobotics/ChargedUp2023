// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.PathWeaverCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
                                DriveSubsystem.oddSquare(MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kControllerDeadband))
                                        * DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveSubsystem.oddSquare(MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kControllerDeadband))
                                        * DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveSubsystem.oddSquare(MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kControllerDeadband))
                                        * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                                false),
                        m_robotDrive));


        m_chooser.addOption("BlueHangerTwoObjectAuton", "BlueHanger TwoObjectAuton");
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
        if (m_chooser.getSelected() != null){
                path = m_chooser.getSelected().split(" ");
                return null;
        }
        else{
                return null;
        }
        
        SequentialCommandGroup twoObjectDropoff = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PathWeaverCommand(m_robotDrive, path[0] + 'twoObjectAuton1', true)
                )
                new ParallelCommandGroup(
                        new PathWeaverCommand(m_robotDrive, path[0] + 'twoObjectAuton2', false)
                )
        )

    }
}
