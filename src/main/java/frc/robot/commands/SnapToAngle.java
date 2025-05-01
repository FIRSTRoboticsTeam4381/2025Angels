// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class SnapToAngle extends Command 
{
    
    public Swerve swerve;
    //public PIDController x;
    //public PIDController y;
    public PIDController r;
    public SnapToAngle(Swerve s){

        swerve = s;
        //x = new PIDController(3.5, 0, 0);
        //y = new PIDController(3.5, 0, 0);
        r = new PIDController(.04, 0, 0);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }
    // Called when the command is initially scheduled
    @Override
    public void initialize(){
  
        Rotation2d angle = swerve.getOdometryYaw();
        Rotation2d target;

        if(angle.getMeasure().isNear(Rotation2d.kZero.getMeasure(), Degrees.of(90)))
        {
            target = Rotation2d.kZero;
        }
        else
        {
            target = Rotation2d.k180deg;
        }
        
        r.setSetpoint(target.getDegrees());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
       swerve.drive(new Translation2d(0,0), getRPower(), true, true, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }

    public double getRPower(){
        return r.calculate(swerve.getPose().getRotation().getDegrees());
    }

}
