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
        //return new PreviewAuto("Test");
        return new PreviewAuto(Commands.none());
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
            return new PreviewAuto("BottomtoEtoD");
        }
        /*public static PreviewAuto BottomtoFtoE() {
            return new PreviewAuto("BottomtoFtoE");
        }*/
        public static PreviewAuto BlueRightFToE() {
            return new PreviewAuto("BlueRightFToE");
        }
        public static PreviewAuto RedRightFToE() {
            return new PreviewAuto("RedRightFToE");
        }
        public static PreviewAuto Middleauto() {
            return new PreviewAuto("Middleauto");
        }
        public static PreviewAuto BottomtoBBtoMB() {
            return new PreviewAuto("BottomtoBBtoMB");
        }
        public static PreviewAuto TopToIToJ() {
            return new PreviewAuto("TopToIToJ");
        }
        public static PreviewAuto TopToIToK() {
            return new PreviewAuto("TopToIToK");
        }
        public static PreviewAuto TopToIToL() {
            return new PreviewAuto("TopToIToL");
        }
        public static PreviewAuto JtoTCoarlstation(){
            return ReefSelectTop("JtoTCoarlstation");
        }
        public static PreviewAuto EtoBCoralStation(){
            return ReefSelectBottom("EtoBCoralStation");
        }
        public static PreviewAuto middleball(){
            return ReefSelectBottom("middle ball");
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
    
    
     public static PreviewAuto ReefSelectTop(String JtoTCoarlstation)
    {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(JtoTCoarlstation),
                new ConditionalCommand( new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
                new SequentialCommandGroup(
                    RobotContainer.getRobot().advancedCommands.autointake(),
                    new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To A"))),
                            Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To B"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To G"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To H"))),
                            Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To I"))),
                            Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To J"))),
                            Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To K"))),
                            Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("TCoral Station To L")))
                        ), Autos::chosenReef),
                   RobotContainer.getRobot().advancedCommands.placel4(),
                   new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("A To TCoral Station"))),
                            Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("B To TCoral Station"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("G To TCoral Station"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("H To TCoral Station"))),
                            Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("I To TCoral Station"))),
                            Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("J To TCoral Station"))),
                            Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("K To TCoral Station"))),
                            Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("L To TCoral Station")))
                        ), Autos::chosenReef),
                        
                        new InstantCommand(()-> reefToGo.remove())
                ), reefToGo::isEmpty).repeatedly()



            ), JtoTCoarlstation);
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

    public static PreviewAuto ReefSelectBottom(String EtoBCoralStation)
    {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(EtoBCoralStation),
                new ConditionalCommand( new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
                new SequentialCommandGroup(
                    RobotContainer.getRobot().advancedCommands.autointake(),
                    new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to A"))),
                            Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to B"))),
                            Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to C"))),
                            Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to D"))),
                            Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to E"))),
                            Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to F"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to G"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("BCoral Station to H")))
                        ), Autos::chosenReef),
                   RobotContainer.getRobot().advancedCommands.placel4(),
                   new SelectCommand<Character>(
                        Map.ofEntries(
                            Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("A to BCoral Station"))),
                            Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("C to BCoral Station"))),
                            Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("D to BCoral Station"))),
                            Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("E to BCoral Station"))),
                            Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("F to BCoral Station"))),
                            Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("G to BCoral Station"))),
                            Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("H to BCoral Station")))

                        ), Autos::chosenReef),
                       
                        new InstantCommand(()-> reefToGo.remove())
                ), reefToGo::isEmpty).repeatedly()



            ), EtoBCoralStation);
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
