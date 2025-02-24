package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class SlowDispanse extends Command {
    
    public SlowDispanse(){
        addRequirements(Dispenser.getInstance());
    }

    @Override
    public void initialize(){
      Dispenser.getInstance().dispenseCoralSlow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
      Dispenser.getInstance().stop();
    }
}
