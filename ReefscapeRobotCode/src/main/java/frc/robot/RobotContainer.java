// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Climber.CloseClimberCommand;
import frc.robot.Commands.Climber.OpenClimberCommand;
import frc.robot.Commands.Dispenser.DispenseCoralCommand;
import frc.robot.Commands.Dispenser.PullBackCoralCommand;
import frc.robot.Commands.Elevator.MoveElevatorTo;
import frc.robot.Commands.Elevator.MoveElevatorToSelectedLevel;
import frc.robot.Commands.Swerve.AutoDrive.DriveToClosestBranchCommand;
import frc.robot.Commands.Swerve.AutoDrive.DriveToFeederCommand;
import frc.robot.Commands.Swerve.AutoDrive.DriveToSelectedPoseCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand;
import frc.robot.Commands.Swerve.ManualDrive.RotateByCommand;
import frc.robot.Commands.Swerve.ManualDrive.RotateToCommand;
import frc.robot.Commands.Swerve.ManualDrive.TeleopDriveCommand;
import frc.robot.Commands.Swerve.ManualDrive.ChangeTeleopSpeedModeCommand.SpeedMode;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.RobotOperatorController;

public class RobotContainer {

  private static final int CHASSIS_PORT = 0;
  private static final int OPERATOR_PORT = 1;

  //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);
  public static final CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);

  //Triggers
  public static final Trigger operatorA = operator.a();
  public static final Trigger operatorB = operator.b();
  public static final Trigger operatorX = operator.x();
  public static final Trigger operatorY = operator.y();
  public static final Trigger operatorPovUp = operator.povUp();
  public static final Trigger operatorPovRight = operator.povRight();
  public static final Trigger operatorRB = operator.rightBumper();
  public static final Trigger operatorLB = operator.leftBumper();
  public static final Trigger operatorRT = operator.rightTrigger();
  public static final Trigger operatorLT = operator.leftTrigger();
  public static final Trigger operatorStart = operator.start();
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

    //climber
    chassisPovDown.whileTrue(new OpenClimberCommand());
    chassisPovUp.whileTrue(new CloseClimberCommand());
  
    //dispenser
    chassisX.whileTrue(new DispenseCoralCommand());

    chassisBack.onTrue(new InstantCommand(()->{Swerve.getInstance().resetGyro();}));
    chassisPovRight.whileTrue(new InstantCommand(() -> {Dispenser.getInstance().slowDispense();}));
    chassisPovLeft.whileTrue(new DriveToSelectedPoseCommand());

    // chassis.rightStick().onTrue(new RotateToIntake(true));
    // chassis.leftStick().onTrue(new RotateToIntake(false));
    
  }

  
}
