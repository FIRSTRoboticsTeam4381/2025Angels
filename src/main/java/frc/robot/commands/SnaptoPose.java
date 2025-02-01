// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;


public class SnaptoPose extends Command
{
EstimatedRobotPose estimatedPose,
swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), startPose);
}
private Pose2d startPose = new Pose2d(Units.inchesToMeters(177), Units.inchesToMeters(214), Rotation2d.fromDegrees(0));


public Pose2d getPose(){
    return swerveOdometry.getEstimatedPosition();
}
