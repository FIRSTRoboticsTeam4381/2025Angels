// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AdvancedCommands {

  RobotContainer robot;

  /** Creates a new AdvancedCommands. */
  public AdvancedCommands(RobotContainer r) {
    robot = r;
    NamedCommands.registerCommand("l4", l4());
    NamedCommands.registerCommand("l3", l3());
    NamedCommands.registerCommand("l2", l2());
    NamedCommands.registerCommand("l1", l1());
  }

  public Command l4() {

    return new ParallelCommandGroup(
      robot.elevator.level4(),
      robot.pivot.coralScoringTop()
      );

  }

  public Command l3() {

    return new ParallelCommandGroup(
      robot.elevator.level3(),
      robot.pivot.coralScoring()
      );
  }

  public Command l2() {

    return new ParallelCommandGroup(
      robot.elevator.level2(),
      robot.pivot.coralScoring()
      );
  }

  public Command l1() {

    return new ParallelCommandGroup(
      robot.elevator.level1(),
      robot.pivot.intake()
      );
  }

  public Command placel4(){

    return new SequentialCommandGroup(
      robot.advancedCommands.l4(),
      robot.coralintake.Coralinorout(),
      robot.advancedCommands.l1()
    );
  }

  public Command autointake(){
    return new SequentialCommandGroup(
      robot.advancedCommands.l1(),
      robot.coralintake.Coralinorout()
    );
  }

}
