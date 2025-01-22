package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

/** An Claw command that uses the Claw subsystem. */
public class ClawCommand extends Command {
  private final ClawSubsystem m_ClawSubsystem;

  /**
   * Create a new TurntableCommand.
   *
   * @param t_Subsystem The subsystem used by this command.
   */
  public ClawCommand(ClawSubsystem c_Subsystem) {
    m_ClawSubsystem = c_Subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
