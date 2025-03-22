package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LedStrip;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.LedStrip.LedPattern;

public class WaitUntilCoralIsInCommand extends Command {
    @Override
    public void initialize() {
        addRequirements(Dispenser.getInstance());
        Dispenser.getInstance().stop();
        LedStrip.getInstance().setLedPattern(LedPattern.READY_FOR_CORAL);
    }

    @Override
    public void end(boolean interrupted) {
      LedStrip.getInstance().setLedPattern(LedPattern.DEFAULT_COLOR);
    }
  

    @Override
    public boolean isFinished() {
        return Dispenser.getInstance().isAtEntry();
    }
}
