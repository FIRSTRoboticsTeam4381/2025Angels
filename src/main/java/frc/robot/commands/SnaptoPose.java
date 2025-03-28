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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class SnaptoPose extends Command 
{
    // Offset between the reefs
    // 8569.325 mm from other reef
    private final double REEF_TO_REEF = 8.569325;

    public final ArrayList<Pose2d> snapPositions= new ArrayList<Pose2d>(){{
        
        // Blue snaps
        add(new Pose2d(3.559, 2.738, new Rotation2d(Radians.convertFrom(59.62, Degrees))));//blue C
        add(new Pose2d(2.949, 4.175, new Rotation2d(Radians.convertFrom(1, Degrees))));//blue a
        add(new Pose2d(6.06, 3.88, new Rotation2d(Radians.convertFrom(180, Degrees))));//blue g
        add(new Pose2d(3.85, 5.43, new Rotation2d(Radians.convertFrom(-60.17, Degrees))));//blue k
        add(new Pose2d(5.382, 5.273, new Rotation2d(Radians.convertFrom(-119.12, Degrees))));//blue i
        add(new Pose2d(5.98, 3.86, new Rotation2d(Radians.convertFrom(175.36, Degrees))));//blue g
        add(new Pose2d(5.45, 2.77, new Rotation2d(Radians.convertFrom(124.59, Degrees))));//blue f
        add(new Pose2d(3.91, 2.58, new Rotation2d(Radians.convertFrom(63.16, Degrees))));//blue d
        add(new Pose2d(2.95, 3.80, new Rotation2d(Radians.convertFrom(3.09, Degrees))));//blue b
        add(new Pose2d(3.56, 5.24, new Rotation2d(Radians.convertFrom(-57.24, Degrees))));//blue l
        add(new Pose2d(5.11, 5.47, new Rotation2d(Radians.convertFrom(-121.06, Degrees))));//blue j
        add(new Pose2d(5.99, 4.16, new Rotation2d(Radians.convertFrom(176.9, Degrees))));//blue h
        add(new Pose2d(5.23, 2.67, new Rotation2d(Radians.convertFrom(127.89, Degrees))));//blue e
        
        // Red snaps
        // POSITION LETTERS MAY BE WRONG!
        add(new Pose2d(3.559+REEF_TO_REEF, 2.738, new Rotation2d(Radians.convertFrom(59.62, Degrees))));//red ?
        add(new Pose2d(11.49, 4.13, new Rotation2d(Radians.convertFrom(1, Degrees))));//red g
        add(new Pose2d(14.56, 3.88, new Rotation2d(Radians.convertFrom(180, Degrees))));//red a
        add(new Pose2d(12.43, 5.465, new Rotation2d(Radians.convertFrom(-60.17, Degrees))));//red e
        add(new Pose2d(13.94, 5.26, new Rotation2d(Radians.convertFrom(-119.12, Degrees))));//red c
        add(new Pose2d(5.98+REEF_TO_REEF, 3.86, new Rotation2d(Radians.convertFrom(175.36, Degrees))));//red ?
        add(new Pose2d(14.06, 2.86, new Rotation2d(Radians.convertFrom(124.59, Degrees))));//red L
        add(new Pose2d(12.52, 2.61, new Rotation2d(Radians.convertFrom(63.16, Degrees))));//red d
        add(new Pose2d(11.5, 3.77, new Rotation2d(Radians.convertFrom(3.09, Degrees))));//red h
        add(new Pose2d(12.05, 5.29, new Rotation2d(Radians.convertFrom(-57.24, Degrees))));//red f
        add(new Pose2d(13.64, 5.41, new Rotation2d(Radians.convertFrom(-121.06, Degrees))));//red d
        add(new Pose2d(14.63, 4.16, new Rotation2d(Radians.convertFrom(176.9, Degrees))));//red b
        add(new Pose2d(13.75, 2.67, new Rotation2d(Radians.convertFrom(127.89, Degrees))));//red k
        
        
        
        
        
        //Blue Top Starting Pose
        //add(new Pose2d(7.333, 4.025, new Rotation2d(Radians.convertFrom(180, Degrees))));

        //Red Top starting Pose
        //add(new Pose2d(10, 0.25, new Rotation2d(Radians.convertFrom(0, Degrees))));

        //I and J
        //add(new Pose2d(12.32, 2.75, new Rotation2d(Radians.convertFrom(65, Degrees))));

        // G and H
        //add(new Pose2d(11.49, 4, new Rotation2d(Radians.convertFrom(-4, Degrees))));

        // F and E
        //add(new Pose2d(12.31, 5.30, new Rotation2d(Radians.convertFrom(-58, Degrees))));

        //D and C
        //add(new Pose2d(13.85, 5.37, new Rotation2d(Radians.convertFrom(-118, Degrees))));

        // A and B
        //add(new Pose2d(14.63, 3.97, new Rotation2d(Radians.convertFrom(180, Degrees))));

        // K and L
        //add(new Pose2d(13.82, 2.66, new Rotation2d(Radians.convertFrom(120, Degrees))));

        // Coral pickups
        add(new Pose2d(1.57, 7.24, new Rotation2d(Radians.convertFrom(127.25, Degrees))));//blue left
        add(new Pose2d(1.48, 0.74, new Rotation2d(Radians.convertFrom(-128.79, Degrees))));//blue right
        add(new Pose2d(15.97, 0.71, new Rotation2d(Radians.convertFrom(-55.33, Degrees))));//red left
        add(new Pose2d(16.00, 7.28, new Rotation2d(Radians.convertFrom(53.32, Degrees))));//red right
        // Red left
        //add(new Pose2d(16.77, 1.38, new Rotation2d(Radians.convertFrom(-53.95, Degrees))));
        //add(new Pose2d(15.71, 0.74, new Rotation2d(Radians.convertFrom(-54.29, Degrees))));
        
    
    }};
    public Swerve swerve;
    private Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;
    public SnaptoPose(Swerve s){

        swerve = s;
        x = new PIDController(3.5, 0, 0);
        y = new PIDController(3.5, 0, 0);
        r = new PIDController(.04, 0, 0);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        Pose2d currentpose = swerve.getPose();
        // target = currentpose.nearest(snapPositions);

        double bestDistance = Double.MAX_VALUE;

        for(Pose2d p : snapPositions){
            double distance = currentpose.getTranslation().getDistance(p.getTranslation());
            if (distance < bestDistance){
                target = p;
                bestDistance = distance;
            }
        }

        swerve.field.getObject("SnapToPose Target").setPose(target);
        
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
        return false;
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
