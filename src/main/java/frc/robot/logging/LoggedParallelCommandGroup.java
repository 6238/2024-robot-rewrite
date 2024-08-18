package frc.robot.logging;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LoggedParallelCommandGroup extends Command {
    private final Map<Command, Boolean> m_commands = new HashMap<>();
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The
     * command composition will finish when the last command finishes. If the composition is
     * interrupted, only the commands that are still running will be interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    public LoggedParallelCommandGroup(Command... commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public final void addCommands(Command... commands) {
        if (m_commands.containsValue(true)) {
        throw new IllegalStateException(
            "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
                throw new IllegalArgumentException(
                    "Multiple commands in a parallel composition cannot require the same subsystems");
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
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                if (commandRunning.getValue()) {
                    commandRunning.getKey().end(true);
                    Logger.logCommandLifetimeMessage(commandRunning.getKey(), "INTERRUPT");
                }
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return !m_commands.containsValue(true);
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
