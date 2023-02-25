package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Snap90Command extends CommandBase {
    private DriveSubsystem m_subsystem;
    private PIDController snapPID = new PIDController(0, 0, 0); //TODO: tune this

    public Snap90Command(DriveSubsystem subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        double snapAngle = m_subsystem.getPose().getRotation().getDegrees();
        snapAngle = MathUtil.inputModulus(snapAngle, 0, 360);

        //Snap angle
        if (snapAngle >= 315 && snapAngle < 45) snapAngle = 0;
        else if (snapAngle >= 45 && snapAngle < 135) snapAngle = 90;
        else if (snapAngle >= 135 && snapAngle < 225) snapAngle = 180;
        else snapAngle = 270;
    
        //Convert to radians [-pi, pi]
        snapAngle = MathUtil.angleModulus(Math.toRadians(snapAngle));

        //Use heading correction to snap to angle
        m_subsystem.drive(0, 0, snapPID.calculate(m_subsystem.getPose().getRotation().getDegrees(), snapAngle), false);
    }
}