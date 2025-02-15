package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber.Climber;

public class OpenClimberCommand extends Command{

    @Override
    public void initialize() {
        Climber.getInstance().open();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Climber.getInstance().stop();
    }
    
}
