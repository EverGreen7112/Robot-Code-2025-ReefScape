package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class SlowDispenseCommand extends Command {

  public SlowDispenseCommand() {
    addRequirements(Dispenser.getInstance());
  }

  @Override
  public void initialize(){
    Dispenser.getInstance().slowDispense();
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

