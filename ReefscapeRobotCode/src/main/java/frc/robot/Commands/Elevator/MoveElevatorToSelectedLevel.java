package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;

public class MoveElevatorToSelectedLevel extends Command {
    private int m_elevatorLevel;

    public MoveElevatorToSelectedLevel(double elevetorLevel){
        m_elevatorLevel = (int)elevetorLevel;
    }

    @Override
    public void initialize() {
        ElevatorLevel elevatorLevel;
        switch(m_elevatorLevel){
            case 1 :
                elevatorLevel = ElevatorLevel.L1;
                break;
            case 2 :
                elevatorLevel = ElevatorLevel.L2;
                break;
            case 3 :
                elevatorLevel = ElevatorLevel.L3;
                break;
            case 4 :
                elevatorLevel = ElevatorLevel.L4;
                break;
            default:
                elevatorLevel = ElevatorLevel.L1;
        }
        (new MoveElevatorTo(elevatorLevel)).schedule();
    }
}
