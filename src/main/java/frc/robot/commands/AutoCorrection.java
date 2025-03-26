// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class AutoCorrection extends Command 
{
    
    
    public Swerve swerve;
    public static Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;
    public AutoCorrection(Swerve s){

        swerve = s;
        x = new PIDController(3, 0, 0);
        y = new PIDController(3
        , 0, 0);
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

       SmartDashboard.putNumber("Autocorrect/dist",((swerve.getPose().getTranslation().getDistance(target.getTranslation())) ));
       SmartDashboard.putNumber("Autocorrect/angle", (swerve.getPose().getRotation().minus(target.getRotation())).getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        swerve.drive(new Translation2d(0,0), 0, true, true, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return ((swerve.getPose().getTranslation().getDistance(target.getTranslation())) <= 0.035 && 
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
