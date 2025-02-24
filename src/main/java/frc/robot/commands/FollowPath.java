// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;



import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPath extends Command {
  /** Creates a new FollowPath. */
  PIDController xController = new PIDController(1.5, 0, 0);
  PIDController yController = new PIDController(1.5, 0, 0);
  PIDController headingController = new PIDController(3, 0, 0);

  DriveTrain driveTrain;
  Optional<Trajectory<SwerveSample>> trajectory;
  Timer t;
  boolean isRedAlliance = false;
  public FollowPath(Optional<Trajectory<SwerveSample>> trajectory, DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(dt);
    this.driveTrain = dt;
    this.trajectory = trajectory;
    isRedAlliance = DriverStation.getAlliance().get() ==  Alliance.Red;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t = new Timer();
    t.start();
    System.out.println("Trajectory " + trajectory );
    Optional<Pose2d> initialPose = trajectory.get().getInitialPose( isRedAlliance  );
    driveTrain.setPose2D(initialPose.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    Optional<SwerveSample> s = trajectory.get().sampleAt( t.get(),  isRedAlliance );
    SwerveSample sample = s.get();

    Pose2d pose = driveTrain.getPose2d();

    double a = sample.vx + xController.calculate(pose.getX(), sample.x);
    double b = sample.vy + yController.calculate(pose.getY(), sample.y);
    double c = sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading);

    //ChassisSpeeds speeds = new ChassisSpeeds( a, b, c   ); 
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(a, b, c, driveTrain.getGyroscopeRotation());

    System.out.println(" doing " + a + "," + b + "," + c + "  :: " + sample.heading + " // " + t.get() );
    driveTrain.followPath(speeds);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.get() > 2;
  }
}
