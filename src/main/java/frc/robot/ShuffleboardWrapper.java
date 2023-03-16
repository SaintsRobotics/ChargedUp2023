package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.ArmSubsystem;

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

  private static ArmSubsystemExtractor arm;

  /**
   * Creates a new Smartdashboard wrapper
   * @param name Tab name
   */
  public ShuffleboardWrapper(String name){
    m_name = name;
    Shuffleboard.getTab(m_name).getLayout("Match Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 3);
    Shuffleboard.getTab(m_name).getLayout("Match Info").addNumber("Time", () -> MathUtil.clamp(DriverStation.getMatchTime(), 0, 255));
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Enabled", () -> DriverStation.isEnabled());
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Driver Controller", () -> DriverStation.isJoystickConnected(0));
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Operator Controller", () -> DriverStation.isJoystickConnected(1));
  }

  /**
   * Adds a subsystem to the shuffleboard output
   * @param sys The subsystem
   */
  public void addSubsystem(ArmSubsystem sys){
    arm  = (ArmSubsystemExtractor)sys;
    Shuffleboard.getTab(m_name).getLayout("Arm", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 3);
    Shuffleboard.getTab(m_name).getLayout("Arm").addNumber("Extension", () -> arm.getExtension());
    Shuffleboard.getTab(m_name).getLayout("Arm").addNumber("Angle", () -> arm.getPivot());
  }

  public void addCamera(PhotonCamera cam){
    Shuffleboard.getTab(m_name).getLayout("Camera", BuiltInLayouts.kGrid).withPosition(0, 3).withSize(2, 2);
    Shuffleboard.getTab(m_name).getLayout("Camera").addCamera("Robot", "MS HD", "http://127.0.0.1:1181/stream.mjpg", "http://photonvision.local:1181/stream.mjpg");
  }
}
