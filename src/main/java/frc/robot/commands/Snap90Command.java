package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Snap90Command extends CommandBase {
    private DriveSubsystem m_subsystem;
    private PIDController snapPID = new PIDController(7.5, 0, 0); //TODO: tune this
    double snapAngle;
    double currAngle;

    public Snap90Command(DriveSubsystem subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        currAngle = m_subsystem.getPose().getRotation().getDegrees();
        currAngle = MathUtil.inputModulus(currAngle, 0, 360);

        //Snap angle
        if (currAngle >= 315 || currAngle < 45) snapAngle = 0;
        else if (currAngle >= 45 && currAngle < 135) snapAngle = Math.PI/2;
        else if (currAngle >= 135 && currAngle < 225) snapAngle = Math.PI;
        else snapAngle = -Math.PI/2;

    }

    @Override
    public void execute() {
    
        SmartDashboard.putNumber("snap angle", snapAngle);

        //Use heading correction to snap to angle
        m_subsystem.drive(0, 0, snapPID.calculate(m_subsystem.getPose().getRotation().getRadians(), snapAngle), false);
    }
}