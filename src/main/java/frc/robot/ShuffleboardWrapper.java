package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ShuffleboardWrapper {
  private static ShuffleboardWrapper m_default = new ShuffleboardWrapper("Default");
  
  /**
   * Gets the default instance
   * @return The default instance
   */
  public static ShuffleboardWrapper getDefault() {return m_default;}

  private final String m_name;

  private class ArmSubsystemExtractor extends ArmSubsystem {
    public double getExtension() { return m_elevatorMotor.getEncoder().getPosition(); }
    public double getPivot() {return m_pivotEncoder.getAbsolutePosition(); }
  }

  private class DriveSubsystemExtractor extends DriveSubsystem {
    public Field2d getField() {return m_field; }
  }

  private class GrabberSubsystemExtractor extends GrabberSubsystem {
    public double getPSI() { return pHub.getPressure(0); }
    public boolean isOpen() { return false; } //TODO: implement this
  }

  private static ArmSubsystemExtractor arm;
  private static GrabberSubsystemExtractor grabber;
  private static DriveSubsystemExtractor drive;

  /**
   * Creates a new Smartdashboard wrapper
   * @param name Tab name
   */
  public ShuffleboardWrapper(String name){
    m_name = name;
    ShuffleboardLayout matchLay = Shuffleboard.getTab(m_name).getLayout("Match Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 3);
    matchLay.addNumber("Time", () -> MathUtil.clamp(DriverStation.getMatchTime(), 0, 255));
    matchLay.addBoolean("Enabled", () -> DriverStation.isEnabled());
    matchLay.addBoolean("Driver Controller", () -> DriverStation.isJoystickConnected(0));
    matchLay.addBoolean("Operator Controller", () -> DriverStation.isJoystickConnected(1));
  }

  /**
   * Adds a subsystem to the shuffleboard output
   * @param sys The subsystem
   */
  public void addSubsystem(ArmSubsystem sys){
    arm  = (ArmSubsystemExtractor)sys;
    ShuffleboardLayout armLay = Shuffleboard.getTab(m_name).getLayout("Arm", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 3);
    armLay.addNumber("Extension (m)", () -> arm.getExtension());
    armLay.addNumber("Angle (deg)", () -> arm.getPivot());
  }

  public void addCamera(PhotonCamera cam){
    ShuffleboardLayout camLay = Shuffleboard.getTab(m_name).getLayout("Camera", BuiltInLayouts.kGrid).withPosition(0, 3).withSize(2, 2);
    camLay.addCamera("Robot", "MS HD", "http://127.0.0.1:1181/stream.mjpg", "http://photonvision.local:1181/stream.mjpg", "http://10.18.99:1181/stream.mjpg", "http://172.11.22.1:1181/stream.mjpg", "http://172.22.11.2:1181/stream.mjpg");
  }

  public void addSubsystem(DriveSubsystem sys) {
    drive = (DriveSubsystemExtractor)sys;
    ShuffleboardLayout driveLay = Shuffleboard.getTab(m_name).getLayout("Drive", BuiltInLayouts.kGrid).withPosition(2, 3).withSize(2, 2);
    driveLay.add(drive.getField());
  }

  public void addSubsystem(GrabberSubsystem sys){
    grabber = (GrabberSubsystemExtractor)sys;
    ShuffleboardLayout grabberLay = Shuffleboard.getTab(m_name).getLayout("Grabber", BuiltInLayouts.kList).withPosition(3, 2).withSize(1, 1);
    grabberLay.addDouble("PSI", () -> grabber.getPSI());
    grabberLay.addBoolean("Open?", () -> grabber.isOpen());
  }
}
