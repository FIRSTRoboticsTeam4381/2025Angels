// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import org.ejml.equation.Variable;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveLogger;


public class AutoCorrection extends Command 
{
    
     
    public Swerve swerve;
    public static Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;
    public AutoCorrection(Swerve s){

        swerve = s;
        x = new PIDController(2, 0, 0);
        y = new PIDController(2, 0, 0);
        r = new PIDController(.04, 0, 0);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        Pose2d currentpose = swerve.getPose();
        
                // target = currentpose.nearest(snapPositions);
                /*for(Pose2d p : snapPositions){
                    double distance = currentpose.getTranslation().getDistance(p.getTranslation());
                    if (distance < bestDistance){
                        target = p;
                        bestDistance = distance;
                    }
                }*/

        
                swerve.field.getObject("AutoCorrection Target").setPose(target);
                
                x.setSetpoint(target.getX());
                y.setSetpoint(target.getY());
                r.setSetpoint(target.getRotation().getDegrees());
        
            }
        
            // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
       swerve.drive(new Translation2d(-getXPower(),-getYPower()), getRPower(), true, true, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return ((swerve.getPose().getTranslation().getDistance(target.getTranslation())) <= 0.0127 && 
        Math.abs((swerve.getPose().getRotation().minus(target.getRotation())).getDegrees()) <= 1);
    }

    public double getXPower(){
        return x.calculate(swerve.getPose().getX());
    }

    public double getYPower(){
        return y.calculate(swerve.getPose().getY());
    }

    public double getRPower(){
        return r.calculate(swerve.getPose().getRotation().getDegrees());
    }

}
