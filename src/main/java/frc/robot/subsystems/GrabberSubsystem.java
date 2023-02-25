// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private PneumaticHub pHub;
    private DoubleSolenoid solenoid0;
    public boolean enableCompressor = true;

    public GrabberSubsystem() {
        pHub = new PneumaticHub(GrabberConstants.kPneumaticsHubID);
        solenoid0 = new DoubleSolenoid(GrabberConstants.kPneumaticsHubID, PneumaticsModuleType.REVPH,
                GrabberConstants.kIntakeLeftSolenoidPort, GrabberConstants.kIntakeRightSolenoidPort);
        pHub.disableCompressor();
        
        if (enableCompressor) {
            pHub.enableCompressorAnalog(GrabberConstants.kCompressorMinimumPressure, GrabberConstants.kCompressorMaximumPressure);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Pressure", pHub.getPressure(0));
        SmartDashboard.putBoolean("Compressor Enabled", pHub.getCompressor());
    }

    public void off() {
        solenoid0.set(kOff);
    }

    public void forward() {
        solenoid0.set(kForward);
    }

    public void reverse() {
        solenoid0.set(kReverse);
    }

    public void toggle() {
        solenoid0.toggle();
    }

    public void toggleCompressor() {
        enableCompressor = !enableCompressor;
    }
}