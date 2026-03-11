package frc.robot.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;

/**
 * Optimized sequential command group (Team 254). Runs multiple commands per execute cycle instead
 * of one per scheduler cycle. Standard composition rules apply.
 */
public class ChezySequenceCommandGroup extends Command {
  private final List<Command> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  /**
   * Creates a new ChezySequenceCommandGroup. The given commands will be run sequentially, with the
   * composition finishing when the last command finishes.
   *
   * @param commands the commands to include in this composition.
   */
  @SuppressWarnings("this-escape")
  public ChezySequenceCommandGroup(Command... commands) {
    addCommands(commands);
  }

  /**
   * Adds the given commands to the group.
   *
   * @param commands Commands to add, in order of execution.
   */
  @SuppressWarnings("PMD.UseArraysAsList")
  public final void addCommands(Command... commands) {
    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      m_commands.add(command);
      addRequirements(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    m_currentCommandIndex = 0;

    if (!m_commands.isEmpty()) {
      m_commands.get(0).initialize();
    }
  }

  @Override
  public final void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    // Limit loops per execute cycle to prevent watchdog timeout
    // This allows multiple instant commands to run in one cycle
    final int kMaxLoopsPerCycle = 3;
    int loops = 0;

    while (loops < kMaxLoopsPerCycle && m_currentCommandIndex < m_commands.size()) {
      Command currentCommand = m_commands.get(m_currentCommandIndex);

      currentCommand.execute();
      if (currentCommand.isFinished()) {
        currentCommand.end(false);
        m_currentCommandIndex++;
        if (m_currentCommandIndex < m_commands.size()) {
          m_commands.get(m_currentCommandIndex).initialize();
          loops++;
          // Continue loop to run next command
        } else {
          break; // Sequence complete
        }
      } else {
        break; // Current command not finished, wait for next cycle
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (interrupted
        && !m_commands.isEmpty()
        && m_currentCommandIndex > -1
        && m_currentCommandIndex < m_commands.size()) {
      m_commands.get(m_currentCommandIndex).end(true);
    }
    m_currentCommandIndex = -1;
  }

  @Override
  public final boolean isFinished() {
    return m_currentCommandIndex == m_commands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
  }
}
