// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto1(DriveTrain dt,Elevator el,  Optional<Trajectory<SwerveSample>> segmentA, Optional<Trajectory<SwerveSample>> segmentB , Optional<Trajectory<SwerveSample>> segmentC ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println(segmentA);
    System.out.println(segmentB);
    System.out.println(segmentC);
    
    addCommands( 
      //new FollowPath(segmentA, dt) );
      //new Elevator_Levels(el, 3, 0, 0), 
      //new WaitCommand(2),
      //new Elevator_Levels(el, 1, 0, 0),
      new FollowPath(segmentB, dt)); // following paths should be here
  }
}
