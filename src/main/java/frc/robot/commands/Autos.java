// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public final class Autos {

    // TODO register commands in subsystem constructores using
    // NamedCommands.registerCommand()

    // Test autonomous mode
    public static PreviewAuto testAuto() {
        return new PreviewAuto("Test");
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * 
     * @return Autonomous command
     */
    public static PreviewAuto none() {
        return new PreviewAuto(Commands.none());
    }

    // TODO add pathplanner autos here. Example:
    // public static PreviewAuto Front3Note(){
    // return new PreviewAuto("Front3NoteAuto");
    // }

    /*
     * If you want to make a more complex auto using commands,
     * PreviewAuto can also accept (Command, String), which will
     * run Command while still showing a path preview for the path
     * with filename String.
     */

    public static class PreviewAuto {
        public Command auto;
        public ArrayList<Pose2d> preview = new ArrayList<>();

        public void showPreview() {
            if (preview != null) {
                RobotContainer.getRobot().swerve.field.getObject("path").setPoses(preview);
            }
        }

        public PreviewAuto(Command a) {
            auto = a;
        }

        public PreviewAuto(String s) {
            auto = new PathPlannerAuto(s);

            try {
                for (PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s)) {
                    preview.addAll(p.getPathPoses());
                }
            } catch (Exception e) {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }

        public PreviewAuto(Command c, String s) {
            auto = c;

            try {
                for (PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s)) {
                    preview.addAll(p.getPathPoses());
                }
            } catch (Exception e) {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }
    }

        public static PreviewAuto BottomtoEtoD() {
            return new PreviewAuto("Bottom to E to D");
        }

        public static PreviewAuto BottomtoFtoE() {
            return new PreviewAuto("Bottom to F to E");
        }
        public static PreviewAuto BottomBBtoMB() {
            return new PreviewAuto("Bottom to BB to MB");
        }

           public static Queue<Character> reefToGo = new LinkedList<>();
          
           public static Character chosenReef() {
            return reefToGo.peek();
        }
          public static void pickReef() {
        String chooseReef = SmartDashboard.getString("Choose Reef", "");
        reefToGo.clear(); 

        for(String n : chooseReef.split(",")) {
            try {
                reefToGo.add(n.charAt(0));
            }catch(Exception e){}
        }
    }
    
    
     public static PreviewAuto ReefSelectBottom(String autoName)
    {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand( new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
                new SequentialCommandGroup(
                    new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('a', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To A"))),
                            Map.entry('b', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To B"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To G"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To H"))),
                            Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To I"))),
                            Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To J"))),
                            Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To K"))),
                            Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoralstation To L")))
                        ), Autos::chosenReef),
                   RobotContainer.getRobot().advancedCommands.placel4(),
                   new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('a', AutoBuilder.followPath(PathPlannerPath.fromPathFile("A To TCoralstation"))),
                            Map.entry('b', AutoBuilder.followPath(PathPlannerPath.fromPathFile("B To TCoralstation"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("G To TCoralstation"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("H To TCoralstation"))),
                            Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("I To TCoralstation"))),
                            Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("J To TCoralstation"))),
                            Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("K To TCoralstation"))),
                            Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("L To TCoralstation")))
                        ), Autos::chosenReef),
                        RobotContainer.getRobot().advancedCommands.autointake(),
                        new InstantCommand(()-> reefToGo.remove())
                ), reefToGo::isEmpty).repeatedly()



            ), autoName);
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        }
    }

    public static PreviewAuto ReefSelectTop(String autoName)
    {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand( new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
                new SequentialCommandGroup(
                    new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('a', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoralstation to A"))),
                            Map.entry('b', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoralstation to B"))),
                            Map.entry('c', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoralstation to C"))),
                            Map.entry('d', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoralstation to D"))),
                            Map.entry('e', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoralstation to E")))
                        ), Autos::chosenReef),
                   RobotContainer.getRobot().advancedCommands.placel4(),
                   new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('a', AutoBuilder.followPath(PathPlannerPath.fromPathFile("A to BCoralstation"))),
                            Map.entry('b', AutoBuilder.followPath(PathPlannerPath.fromPathFile("B to BCoralstation"))),
                            Map.entry('c', AutoBuilder.followPath(PathPlannerPath.fromPathFile("C to BCoralstation"))),
                            Map.entry('d', AutoBuilder.followPath(PathPlannerPath.fromPathFile("D to BCoralstation"))),
                            Map.entry('e', AutoBuilder.followPath(PathPlannerPath.fromPathFile("E to BCoralstation")))
                        ), Autos::chosenReef),
                        RobotContainer.getRobot().advancedCommands.autointake(),
                        new InstantCommand(()-> reefToGo.remove())
                ), reefToGo::isEmpty).repeatedly()



            ), autoName);
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return none();
        }
    }
    

}
