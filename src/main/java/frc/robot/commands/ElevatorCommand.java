package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.subsystems.ElevatorSubsystem;

/** An elevator command that uses the elevator subsystem. */
public class ElevatorCommand extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final ShooterState m_ShooterState;

  /**
   * Create a new ElevatorCommand.
   *
   * @param e_Subsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem e_Subsystem, ShooterState shooterState) {
    m_ElevatorSubsystem = e_Subsystem;
    m_ShooterState = shooterState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.SetHeight(m_ShooterState.mode.height);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
