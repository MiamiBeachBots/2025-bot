package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {
  public enum ShooterMode {
    DEFAULT,
    INTAKE,
    TROUGH,
    REEFT2,
    REEFT3,
    REEFT4,
    BARGE,
  };

  public static final class Speeds { // TODO
    public static final int DEFAULT = 0;
    public static final int INTAKE = 0;
    public static final int TROUGH = 0;
    public static final int REEFT2 = 0;
    public static final int REEFT3 = 0;
    public static final int REEFT4 = 0;
    public static final int BARGE = 0;
  }

  public final boolean isSensing = false;
  public boolean isLoaded = true;
  public boolean isLowered = true;
  public boolean isResting = true;
  public boolean isShooting = false;
  public boolean axisEnabled = false;
  public ShooterMode mode = ShooterMode.DEFAULT;

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

  public void startShooting() {
    isShooting = true;
  }

  public void stopShooting() {
    isShooting = false;
    if (mode == ShooterMode.INTAKE && isLoaded) {
      mode = ShooterMode.DEFAULT;
    } else if (mode != ShooterMode.INTAKE && !isLoaded) {
      mode = ShooterMode.DEFAULT;
    }
  }

  public void toggleAxis() {
    axisEnabled = !axisEnabled;
  }

  public void setLowered() {
    isLowered = true;
    mode = ShooterMode.DEFAULT;
  }

  public double getShooterSpeed() {

    switch (mode) {
      case TROUGH:
        return Speeds.TROUGH;
      case INTAKE:
        return Speeds.INTAKE;
      case REEFT2:
        return Speeds.REEFT2;
      case REEFT3:
        return Speeds.REEFT3;
      case REEFT4:
        return Speeds.REEFT4;
      case BARGE:
        return Speeds.BARGE;
      default:
        return Speeds.DEFAULT;
    }
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
    SmartDashboard.putBoolean("Arm Shooting", isShooting);
  }
}
