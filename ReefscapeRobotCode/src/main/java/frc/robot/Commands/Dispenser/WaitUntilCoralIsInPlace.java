package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class WaitUntilCoralIsInPlace extends Command {
    @Override
    public void initialize() {
        addRequirements(Dispenser.getInstance());
        Dispenser.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return Dispenser.getInstance().isAtEntry();
    }
}
