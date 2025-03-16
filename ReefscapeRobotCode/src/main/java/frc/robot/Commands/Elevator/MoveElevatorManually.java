package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class MoveElevatorManually extends Command {

    public MoveElevatorManually(){
        addRequirements(Elevator.getInstance());
    }
    

    @Override
    public void initialize(){
        Elevator.getInstance().moveManually(Elevator.MANUAL_ELEVATOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
