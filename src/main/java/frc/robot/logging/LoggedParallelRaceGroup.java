package frc.robot.logging;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LoggedParallelRaceGroup extends Command {
    private final Set<Command> m_commands = new HashSet<>();
    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new ParallelCommandRace. The given commands will be executed simultaneously, and will
     * "race to the finish" - the first command to finish ends the entire command, with all other
     * commands being interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    public LoggedParallelRaceGroup(Command... commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public final void addCommands(Command... commands) {
        if (!m_finished) {
            throw new IllegalStateException(
                "Commands cannot be added to a composition while it's running!");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
                throw new IllegalArgumentException(
                    "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        m_finished = false;
        for (Command command : m_commands) {
            command.initialize();
            Logger.logCommandLifetimeMessage(command, "INITIALIZE");
        }
    }

    @Override
    public final void execute() {
        for (Command command : m_commands) {
            command.execute();
            if (command.isFinished()) {
                m_finished = true;
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        for (Command command : m_commands) {
            command.end(!command.isFinished());
            if (command.isFinished()) {
                Logger.logCommandLifetimeMessage(command, "FINISH");
            } else {
                Logger.logCommandLifetimeMessage(command, "INTERRUPT");
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return m_finished;
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }
}
