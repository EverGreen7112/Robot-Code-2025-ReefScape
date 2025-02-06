package frc.robot.Commands.Dispenser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Dispenser.Dispenser;

public class RemoveAlgeaCommand extends Command {

  public RemoveAlgeaCommand() {
    addRequirements(Dispenser.getInstance());
  }

  @Override
  public void initialize(){
    Dispenser.getInstance().dropAlgea();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Dispenser.getInstance().stop();
  }


    
}
