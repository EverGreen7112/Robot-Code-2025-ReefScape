package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class WaitUntilCoralIsOutCommand extends Command {
    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return !Dispenser.getInstance().isAtEntry() && !Dispenser.getInstance().isAtExit();
    }   
    
}
