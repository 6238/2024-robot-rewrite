package frc.robot.logging;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LoggedParallelDeadlineGroup extends Command {
    private final Map<Command, Boolean> m_commands = new HashMap<>();
    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;
    private Command m_deadline;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new ParallelDeadlineGroup. The given commands, including the deadline, will be
     * executed simultaneously. The composition will finish when the deadline finishes, interrupting
     * all other still-running commands. If the composition is interrupted, only the commands still
     * running will be interrupted.
     *
     * @param deadline the command that determines when the composition ends
     * @param otherCommands the other commands to be executed
     * @throws IllegalArgumentException if the deadline command is also in the otherCommands argument
     */
    public LoggedParallelDeadlineGroup(Command deadline, Command... otherCommands) {
        addCommands(otherCommands);
        setDeadline(deadline);
    }

    /**
     * Sets the deadline to the given command. The deadline is added to the group if it is not already
     * contained.
     *
     * @param deadline the command that determines when the group ends
     * @throws IllegalArgumentException if the deadline command is already in the composition
     */
    public final void setDeadline(Command deadline) {
        @SuppressWarnings("PMD.CompareObjectsWithEquals")
        boolean isAlreadyDeadline = deadline == m_deadline;
        if (isAlreadyDeadline) {
            return;
        }
        if (m_commands.containsKey(deadline)) {
            throw new IllegalArgumentException(
                "The deadline command cannot also be in the other commands!");
        }
        addCommands(deadline);
        m_deadline = deadline;
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public final void addCommands(Command... commands) {
        if (!m_finished) {
        throw new IllegalStateException(
            "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
                throw new IllegalArgumentException(
                    "Multiple commands in a parallel group cannot require the same subsystems");
            }
            m_commands.put(command, false);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            commandRunning.getKey().initialize();
            Logger.logCommandLifetimeMessage(commandRunning.getKey(), "INITIALIZE");
            commandRunning.setValue(true);
        }
        m_finished = false;
    }

    @Override
    public final void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            commandRunning.getKey().execute();
            if (commandRunning.getKey().isFinished()) {
                commandRunning.getKey().end(false);
                Logger.logCommandLifetimeMessage(commandRunning.getKey(), "FINISH");
                commandRunning.setValue(false);
                if (commandRunning.getKey().equals(m_deadline)) {
                    m_finished = true;
                }
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (commandRunning.getValue()) {
                commandRunning.getKey().end(true);
                Logger.logCommandLifetimeMessage(commandRunning.getKey(), "INTERRUPT");
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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addStringProperty("deadline", m_deadline::getName, null);
    }
}
