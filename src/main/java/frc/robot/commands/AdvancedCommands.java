// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    NamedCommands.registerCommand("placel4", placel4());
    NamedCommands.registerCommand("autointake", autointake());
    NamedCommands.registerCommand("AutoCorrect", new AutoCorrection(robot.swerve));
    NamedCommands.registerCommand("lowAlgae", lowAlgae());
    NamedCommands.registerCommand("net", NetAlgaeauto());
    NamedCommands.registerCommand("algaeReef", AlgaeReef());
  }

  public Command l4() {

    /*return new SequentialCommandGroup(
      robot.pivot.pivotAllUp(),
      new WaitCommand(.5),
      robot.elevator.level4(),
      robot.pivot.coralScoringTop()
      ).withName("l4");*/

      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level4(),
        robot.pivot.coralScoringTop()
      );

  }

  /**
   * Version of l4 that doesn't pivot all the way so drivers can line up
   * @return Command
   */
  public Command l4Teleop() {
      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level4(),
        robot.pivot.coralScoring()
      );

  }

  public Command l3() {

    /*return new SequentialCommandGroup(
      robot.pivot.pivotAllUp(),
      new WaitCommand(.5),
      robot.elevator.level3(),
      robot.pivot.coralScoring()
      ).withName("l3");*/

      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level3(),
        robot.pivot.coralScoring()
      );
  }

  public Command l2() {

    /*return new SequentialCommandGroup(
      robot.pivot.pivotAllUp(),
      new WaitCommand(.5),
      robot.elevator.level2(),
      robot.pivot.coralScoring()
      ).withName("l2");*/

      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level2(),
        robot.pivot.coralScoring()
      );
  }

  public Command l3Teleop() {

      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level3(),
        robot.pivot.coralScoringTeleop()
      );
  }

  public Command l2Teleop() {

      return new SequentialCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level2(),
        robot.pivot.coralScoringTeleop()
      );
  }


  public Command l1() {

    return new SequentialCommandGroup(
      //robot.pivot.pivotAllUp(),
      robot.pivot.intake(),
      robot.elevator.level1()
      );

      /*return new ParallelCommandGroup(
        robot.pivot.intake(),
        robot.elevator.level1()
      );*/
  }

  public Command placel4(){

    return new SequentialCommandGroup(
      l4(),
      robot.coralintake.out(),
      l1()
    );
  }

  public Command hold(){
    return new RepeatCommand(new InstantCommand(() -> {}, robot.elevator, robot.pivot));
  }

  public Command autointake(){
    return new SequentialCommandGroup(
      l1(),
      robot.coralintake.coralIn()
    );
  }

  public Command NetAlgae(){
    return new ParallelCommandGroup(robot.pivot.net(),
    robot.elevator.net());
  }

  public Command NetAlgaeauto(){
    return new SequentialCommandGroup(new ParallelCommandGroup(robot.pivot.net(),
    robot.elevator.net()), robot.coralintake.autoOut());
  }

  public Command AlgaeReef(){
    return new ParallelCommandGroup(robot.pivot.Algaereef(),
    robot.elevator.Algaereef());
  }

  public Command processor(){
    return new ParallelCommandGroup(robot.pivot.Proccesor(),
    robot.elevator.level1());
  }

  public Command lollypop(){
    return new ParallelCommandGroup(robot.pivot.lollypoppickup(),
    robot.elevator.lollypopalgae());
  }
  public Command lowAlgae(){
    return new SequentialCommandGroup( new ParallelCommandGroup(robot.pivot.trough(),
    robot.elevator.lowalgae()), robot.coralintake.ManualCoarlIn());
  }
  public Command algaeGround(){
    return new ParallelCommandGroup(robot.pivot.Algaeground(),
     robot.elevator.lollypopalgae());
  }
  public Command algaeL2(){
    return new ParallelCommandGroup(robot.pivot.Algael2(),
    robot.elevator.algael2());
  }
public Command algaeL3(){
return new ParallelCommandGroup(robot.pivot.Algael3(),
robot.elevator.algael3());

}
public Command coralL1(){
  return new ParallelCommandGroup(robot.pivot.Corall1(),
  robot.elevator.corall1());
  
  }
}
