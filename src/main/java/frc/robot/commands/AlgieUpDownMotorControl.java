
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algie_InTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgieUpDownMotorControl extends Command {
  Algie_InTake it;
     boolean of;
     PIDController pid = new PIDController(1, 0, 0);
  /** Creates a new AlgieUpDownMeoto. */
  public AlgieUpDownMotorControl(Algie_InTake it, boolean of) {
    // Use addRequirements() here to declare subsystem dependencies.
     this.it = it;
     this.of = of;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (of) {
      pid.setSetpoint(0);//Here goes the set point to get while taking DOWN our subsystem//
      double s = pid.calculate(it.getPosition());
      it.moveUpDown(s);
    }else{
      pid.setSetpoint(0);//Here goes the set point to get while taking UP our subsystem//
      double s = pid.calculate(it.getPosition());
      it.moveUpDown(s);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
