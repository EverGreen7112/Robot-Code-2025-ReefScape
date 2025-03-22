// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Climber.CloseClimberCommand;
import frc.robot.Commands.Climber.OpenClimberCommand;
import frc.robot.Commands.Dispenser.DispenseCoralCommand;
import frc.robot.Commands.Dispenser.SlowDispenseCommand;
//import frc.robot.Commands.Elevator.MoveElevatorManually;
import frc.robot.Commands.Elevator.MoveElevatorTo;
import frc.robot.Commands.Elevator.MoveElevatorToSelectedLevel;
import frc.robot.Commands.Swerve.AutoDrive.DriveToClosestBranchCommand;
import frc.robot.Commands.Swerve.AutoDrive.DriveToSelectedPoseCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand;
import frc.robot.Commands.Swerve.ManualDrive.TeleopDriveCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand.SpeedMode;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

  private static final int CHASSIS_PORT = 0;

  //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);

  //Triggers
  public static final Trigger chassisStart = chassis.start();
  public static final Trigger chassisBack = chassis.back();
  public static final Trigger chassisA = chassis.a();
  public static final Trigger chassisB = chassis.b();
  public static final Trigger chassisX = chassis.x();
  public static final Trigger chassisY = chassis.y();
  public static final Trigger chassisRT = chassis.rightTrigger();
  public static final Trigger chassisLT = chassis.leftTrigger();
  public static final Trigger chassisRB = chassis.rightBumper();
  public static final Trigger chassisLB = chassis.leftBumper();
  public static final Trigger chassisPovUp = chassis.povUp();
  public static final Trigger chassisPovDown = chassis.povDown();

  //commands
  public static final Trigger chassisPovRight = chassis.povRight();
  public static final Trigger chassisPovLeft = chassis.povLeft();
  public static final TeleopDriveCommand teleopCommand = new TeleopDriveCommand(chassis::getLeftX, chassis::getLeftY, chassis::getRightX);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //chassis
    Swerve.getInstance().setDefaultCommand(teleopCommand);
    chassisRB.whileTrue(new DriveToClosestBranchCommand(true));                                                                                        
    chassisLB.whileTrue(new DriveToClosestBranchCommand(false));

    chassisRT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kTurbo));
    chassisLT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kSlow));
    
    //elevator
    chassisA.whileTrue( new MoveElevatorTo(ElevatorLevel.CLOSED));
    chassisY.onTrue( new MoveElevatorToSelectedLevel());
    //chassisStart.whileTrue( new MoveElevatorManually());

    //climber
    chassisPovDown.whileTrue(new OpenClimberCommand());
    chassisPovUp.whileTrue(new CloseClimberCommand());
  
    //dispenser
    chassisX.whileTrue(new DispenseCoralCommand());

    chassisBack.onTrue(new InstantCommand(()->{Swerve.getInstance().resetGyro();}));
    chassisPovRight.whileTrue(new SlowDispenseCommand());
    chassisPovLeft.whileTrue(new DriveToSelectedPoseCommand());

    
    
  }

  
}
