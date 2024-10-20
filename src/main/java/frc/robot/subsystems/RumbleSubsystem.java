package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleSubsystem extends SubsystemBase {
    private double rumbleTimer;
    private CommandXboxController controller;

    public RumbleSubsystem(CommandXboxController _controller) {
        rumbleTimer = 0;
        controller = _controller;
    }

    public Command startRumble(double time) {
        return runOnce(() -> {
            if (rumbleTimer > time) {
                return;
            }
            rumbleTimer = time;
        });
    }

    public void periodic() {
        rumbleTimer -= 0.05;
        
        if (rumbleTimer > 0) {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        } else {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
