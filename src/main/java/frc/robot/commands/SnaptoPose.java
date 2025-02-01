// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import org.ejml.equation.Variable;
import org.photonvision.EstimatedRobotPose;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class SnaptoPose extends Command
{
    public final ArrayList<Pose2d> snapPositions= new ArrayList<Pose2d>(){{
        new Pose2d(3.764, 5.242, new Rotation2d(Radians.convertFrom(-60, Degrees)));
        new Pose2d(5.203, 5.254, new Rotation2d(Radians.convertFrom(-120, Degrees)));
        new Pose2d(5.970, 4.007, new Rotation2d(Radians.convertFrom(180, Degrees)));
        new Pose2d(5.203, 2.784, new Rotation2d(Radians.convertFrom(120, Degrees)));
        new Pose2d(3.740, 2.784, new Rotation2d(Radians.convertFrom(60, Degrees)));
        new Pose2d(3.045, 4.031, new Rotation2d(Radians.convertFrom(0, Degrees)));
    }};
    public Swerve swerve;
    public SnaptoPose(Swerve s){
        swerve = s;
        addRequirements(swerve);
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        Pose2d currentpose = swerve.getPose();
        currentpose.nearest(snapPositions);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return Math.abs(position - feedback.get()) < error;
    }


}
