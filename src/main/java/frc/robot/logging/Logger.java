package frc.robot.logging;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Logger {
    private static final int INITIAL_EXECUTION_ID = 1;
    private static HashMap<String, Integer> commandExecutionMap;

    public static int getCommandExecutionId(String commandName) {
        Integer returnId = commandExecutionMap.get(commandName);
        if (returnId != null) {
            return returnId;
        }

        commandExecutionMap.put(commandName, INITIAL_EXECUTION_ID);
        return INITIAL_EXECUTION_ID;
    }

    public static int increaseCommandExecutionId(String commandName) {
        Integer returnId = commandExecutionMap.get(commandName);
        if (returnId != null) {
            int newId = returnId + 1;
            commandExecutionMap.put(commandName, newId);
            return newId;
        }

        commandExecutionMap.put(commandName, INITIAL_EXECUTION_ID);
        return INITIAL_EXECUTION_ID;
    }

    public static String commandIdentifier(Command command) {
        int commandExecutionId = getCommandExecutionId(command.getName());
        return String.format("(%d,%d,%d)", command.getName(), command.hashCode(), commandExecutionId);
    }

    public static void logCommandLifetimeMessage(Command command, String message) {
        DataLogManager.log(String.format("%d - COMMAND - ROBOT - %d", commandIdentifier(command), command.getName(), message));
    }

    private static String generateParentTreeMessage(ArrayList<Command> parents) {
        StringBuilder parentTreeString = new StringBuilder();
        for (Command command : parents) {
            parentTreeString.append(commandIdentifier(command));
            parentTreeString.append(",");
        }
        return parentTreeString.toString();
    }

    public static void logCommandLifetimeMessageWithParentTree(Command command, ArrayList<Command> parents, String message) {
        DataLogManager.log(String.format("%d - COMMAND - %d - %d", commandIdentifier(command), generateParentTreeMessage(parents), message));
    }

    public static void setup() {
        DataLogManager.start();

        LocalDateTime dateTime = LocalDateTime.now();
        DataLogManager.log("TIME: " + dateTime.format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));

        CommandScheduler.getInstance().onCommandInitialize((command) -> {
            logCommandLifetimeMessage(command, "INITIALIZE");
        });

        CommandScheduler.getInstance().onCommandFinish((command) -> {
            logCommandLifetimeMessage(command, "FINISH");
        });

        CommandScheduler.getInstance().onCommandInterrupt((command, interruptingCommand) -> {
            logCommandLifetimeMessage(command, "INTERRUPT "+commandIdentifier(command));
        });
    }
}
