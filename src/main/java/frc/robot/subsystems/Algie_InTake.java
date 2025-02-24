// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Algie_InTake extends SubsystemBase {
  /** Creates a new InTake. */
  VictorSPX intake;
  SparkMax UpDownMotor;
  DoubleSolenoid sol;
  double basePosition;
  public double getBasePosition() {
    return basePosition;
  }
  public void setBasePosition(double basePosition) {
    this.basePosition = basePosition;
  }
  public Algie_InTake() {
    intake = new VictorSPX(RobotMap.algieintake);
    sol = new DoubleSolenoid(RobotMap.moduleintake, PneumaticsModuleType.REVPH, RobotMap.forward, RobotMap.reverse);
    setBasePosition(getPosition());
  }
  public void moveIntake(double s){
    intake.set(VictorSPXControlMode.PercentOutput,s);
  }
  public double getPosition(){
    return UpDownMotor.getEncoder().getPosition();
  }
  public void moveUpDown(double s){
    UpDownMotor.set(s);
  }
  public void goDown(){
    sol.set(Value.kForward);
  }
  public void goUp (){
    sol.set(Value.kReverse);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
