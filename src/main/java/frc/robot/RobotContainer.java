// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgieTake;
import frc.robot.commands.AlgieUpDownMotorControl;
import frc.robot.commands.Auto1;
import frc.robot.commands.AlgieDownUpNeumatics;
import frc.robot.commands.Coral_Intake_Move;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Elevator_Levels;
import frc.robot.commands.Elevator_Move;
import frc.robot.subsystems.Algie_InTake;
import frc.robot.subsystems.Coral_InTake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  private final  CommandXboxController m_controller = new CommandXboxController(0);
  private final  CommandXboxController m_controller2 = new CommandXboxController(1);
  private final DriveTrain m_drivetrainSubsystem = new DriveTrain();
  Algie_InTake it = new Algie_InTake();
  Coral_InTake ci = new Coral_InTake();
  Elevator elevator = new Elevator();
  static RobotContainer instance;
  public static RobotContainer getInstance(){
    return instance;
  }
  public CommandXboxController getPilot(){
    return m_controller;
  }
  public CommandXboxController getCoPilot(){
    return m_controller2;
  }
  private final Optional<Trajectory<SwerveSample>> trajectory1_1;
  private final Optional<Trajectory<SwerveSample>> trajectory1_2;
  private final Optional<Trajectory<SwerveSample>> trajectory1_3;


  public RobotContainer() {
    trajectory1_1= Choreo.loadTrajectory("Auto1_1");
    trajectory1_2 = Choreo.loadTrajectory("Auto1_2");
    trajectory1_3 = Choreo.loadTrajectory("Auto1_3");
    ///////////////////////////////////////////////////////////////////////////////////////////////
    DefaultDriveCommand ddc = new DefaultDriveCommand(m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getLeftY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getLeftX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getRightX()) * DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND );
    m_drivetrainSubsystem.setDefaultCommand(ddc);
    //////////////////////////////////////////////////////////////////////////////////////////////
    elevator.setDefaultCommand(new Elevator_Move(elevator, m_controller,m_controller2));
    ///////////CONTROL 1/////////////////////////////////////////////////
    m_controller.a().whileTrue(new Coral_Intake_Move(ci,RobotMap.coralspeed));
    m_controller.b().whileTrue(new Coral_Intake_Move(ci,-RobotMap.coralspeed));
   /*m_controller.rightBumper().whileTrue(new AlgieDownUpNeumatics(it, true));
    m_controller.leftBumper().whileTrue(new AlgieDownUpNeumatics(it, false));*/
    m_controller.a().whileTrue(new AlgieUpDownMotorControl(it, true));
    m_controller.b().whileTrue(new AlgieUpDownMotorControl(it,false));
   m_controller.y().whileTrue(new AlgieTake(it, RobotMap.algiespeed));
   m_controller.x().whileTrue(new AlgieTake(it, -RobotMap.algiespeed));
    ///////////CONTROL 2/////////////////////////////////////////////////
    //ci.setDefaultCommand(new Coral_Intake_Move_M(ci,xController2));
    m_controller2.x().whileTrue(new AlgieTake(it, -RobotMap.algiespeed));
    m_controller2.y().whileTrue(new AlgieTake(it,RobotMap.algiespeed));     
    m_controller2.leftBumper().whileTrue(new AlgieDownUpNeumatics(it, true));
    m_controller2.rightBumper().whileTrue(new AlgieDownUpNeumatics(it, false));

    m_controller2.a().whileTrue(new Coral_Intake_Move(ci,RobotMap.coralspeed));
    m_controller2.b().whileTrue(new Coral_Intake_Move(ci,-RobotMap.coralspeed));

    m_controller2.povDown().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 1, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
    m_controller2.povRight().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 2, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
    m_controller2.povUp().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 3, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
    m_controller2.povLeft().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 4, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
    configureBindings();
    RobotContainer.instance = this;
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new Auto1(m_drivetrainSubsystem,elevator,trajectory1_1,trajectory1_2,trajectory1_3);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
