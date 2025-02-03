package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {
  /** Class for presets */
  public static class ShooterMode {
    public final String name;
    public final int speed;
    public final int height;
    public final int angle;

    public ShooterMode(String Name, int Speed, int Height, int Angle) {
      name = Name;
      speed = Speed;
      height = Height;
      angle = Angle;
    }
  }

  // TODO: Numbers
  public static class ShooterModes {
    public static final ShooterMode DEFAULT = new ShooterMode("Default", 0, 0, 0);
    public static final ShooterMode INTAKE = new ShooterMode("Intake", 0, 0, 0);
    public static final ShooterMode TROUGH = new ShooterMode("Trough", 0, 0, 0);
    public static final ShooterMode REEFT2 = new ShooterMode("ReefT2", 0, 0, 0);
    public static final ShooterMode REEFT3 = new ShooterMode("ReefT3", 0, 0, 0);
    public static final ShooterMode REEFT4 = new ShooterMode("ReefT4", 0, 0, 0);
    public static final ShooterMode BARGE = new ShooterMode("Barge", 0, 0, 0);
  }
  ;

  public final boolean isSensing = false;
  public boolean isLoaded = true;
  public boolean isLowered = true;
  public boolean isResting = true;
  public boolean isShooting = false;
  public boolean axisEnabled = false;
  public ShooterMode mode = ShooterModes.DEFAULT;

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
    if (mode == ShooterModes.INTAKE && isLoaded) {
      mode = ShooterModes.DEFAULT;
    } else if (mode != ShooterModes.INTAKE && !isLoaded) {
      mode = ShooterModes.DEFAULT;
    }
  }

  public void toggleAxis() {
    axisEnabled = !axisEnabled;
  }

  public void setLowered() {
    isLowered = true;
    mode = ShooterModes.DEFAULT;
  }

  public double getShooterSpeed() {
    return mode.speed;
  }

  /**
   * Updates the values on the SmartDashboard related to the shooter state. This method puts the
   * values of various shooter state variables onto the SmartDashboard. The variables include
   * whether the manual arm mode is enabled, the current arm mode, whether the shooter is loaded,
   * whether the arm is lowered, and whether the arm is shooting.
   */
  public void updateDash() {
    SmartDashboard.putBoolean("Manual Arm Mode Enabled", axisEnabled);
    SmartDashboard.putString("Arm Mode", mode.name);
    SmartDashboard.putBoolean("Loaded", isLoaded);
    SmartDashboard.putBoolean("Lowered", isLowered);
    SmartDashboard.putBoolean("Resting", isResting);
    SmartDashboard.putBoolean("Arm Shooting", isShooting);
  }
}
