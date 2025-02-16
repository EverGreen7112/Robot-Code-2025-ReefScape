package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class DispanceCoral extends Command {
    @Override
    public void initialize() {
        addRequirements(Dispenser.getInstance());
        new DispenseCoralCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return new WaitUntilCoralIsInPlace().isFinished();
    }
    
}
