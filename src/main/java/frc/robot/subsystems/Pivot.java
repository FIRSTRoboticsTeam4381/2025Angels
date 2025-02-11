// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged

public class Pivot extends SubsystemBase {

private SparkMax pivotmotor;

  /** Creates a new Pivot. */
  public Pivot() {

    pivotmotor = new SparkMax(52, MotorType.kBrushless);

    SparkMaxConfig pivotmotorConfig = new SparkMaxConfig();

    pivotmotorConfig
    .smartCurrentLimit(40)
    .idleMode(IdleMode.kBrake);

    pivotmotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    pivotmotor.configure(pivotmotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    // Quick and dirty way to enable position logging
    // The line is a no-op here but enables the desired packets
    pivotmotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }

  public Command joystickcontrol(Supplier<Double> joystickMove)
  {
    return 
      new InstantCommand(()-> pivotmotor.set(-joystickMove.get()),this).repeatedly()
    ;
  }

  public Command goToPosition(double target, double range){
    return new SparkPosition(pivotmotor, target, range, this).withName("goToPose");
  }

  public Command coralScoring()
  {
    return goToPosition(0, 0);
  }
  
  public Command coralScoringTop()
  {
    return goToPosition(0,0);
  }

  public Command intake()
  {
    return goToPosition(0,0);
  }

}
