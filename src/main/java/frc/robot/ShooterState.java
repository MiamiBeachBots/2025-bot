package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {

  public boolean isLoaded = true;
  public boolean isLowered = true;
  public boolean isResting = true;

  public enum ShooterMode {
    DEFAULT,
    INTAKE,
    TROUGH,
    REEF1,
    REEF2,
    REEF3,
    BARGE,
  };

  public static final class Speeds {
    public static final int DEFAULT = 0;
    public static final int INTAKE = 0;
    public static final int TROUGH = 0;
    public static final int REEFT2 = 0;
    public static final int REEFT3 = 0;
    public static final int REEFT4 = 0;
    public static final int BARGE = 0;
  }

  public ShooterMode mode = ShooterMode.DEFAULT;
  public boolean axisEnabled = false;
  public boolean shooting = false;

  public ShooterState() {}

  public void setLoaded() {
    isLoaded = true;
  }

  public void setMode(ShooterMode newMode) {
    mode = newMode;
  }

  public void setResting() {
    isResting = true;
  }

  public void toggleAxis() {
    axisEnabled = !axisEnabled;
  }

  public void setFired() {
    isLoaded = false;
    isLowered = false;
    mode = ShooterMode.DEFAULT;
  }

  public void setLowered() {
    isLowered = true;
    mode = ShooterMode.DEFAULT;
  }

  public double getShooterSpeed() {

    switch (mode) {
      case TROUGH: // TODO
        return Speeds.TROUGH;
      case INTAKE:
        return Speeds.INTAKE;
      case REEF1:
        return Speeds.REEFT2;
      case REEF2:
        return Speeds.REEFT3;
      case REEF3:
        return Speeds.REEFT4;
      default:
        return Speeds.DEFAULT;
    }

    // return 0;
  }

  /**
   * Updates the values on the SmartDashboard related to the shooter state. This method puts the
   * values of various shooter state variables onto the SmartDashboard. The variables include
   * whether the manual arm mode is enabled, the current arm mode, whether the shooter is loaded,
   * whether the arm is lowered, and whether the arm is shooting.
   */
  public void updateDash() {
    SmartDashboard.putBoolean("Manual Arm Mode Enabled", axisEnabled);
    SmartDashboard.putString("Arm Mode", mode.name());
    SmartDashboard.putBoolean("Loaded", isLoaded);
    SmartDashboard.putBoolean("Lowered", isLowered);
    SmartDashboard.putBoolean("Resting", isResting);
    SmartDashboard.putBoolean("Arm Shooting", shooting);
  }
}
