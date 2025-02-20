package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Utils.RobotOperatorController;

public class MoveElevatorToSelectedLevel extends Command {
    private int m_elevatorLevel;

    public MoveElevatorToSelectedLevel(){
        
    }

    @Override
    public void initialize() {
        ElevatorLevel elevatorLevel;
        m_elevatorLevel = (int)RobotOperatorController.getInstance().getElevatorLevel();

        switch(m_elevatorLevel){
            case 0:
                elevatorLevel = ElevatorLevel.CLOSED;
                break;
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
                elevatorLevel = ElevatorLevel.CLOSED;
        }
        (new MoveElevatorTo(elevatorLevel)).schedule();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
