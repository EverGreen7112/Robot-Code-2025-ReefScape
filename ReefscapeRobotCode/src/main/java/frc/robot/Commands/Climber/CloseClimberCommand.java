package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber.Climber;

public class CloseClimberCommand extends Command{

    @Override
    public void initialize() {
        Climber.getInstance().close();
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
