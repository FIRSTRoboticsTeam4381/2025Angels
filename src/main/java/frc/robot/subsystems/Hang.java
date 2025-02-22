// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Hang extends SubsystemBase {
  // **creating the variables for the motors**
  private SparkFlex hangmotor1;
  //private SparkMax hangmotor2;

  public Hang() {
    // assign cAn ID and Motor type
    hangmotor1 = new SparkFlex(58, MotorType.kBrushless);
    //hangmotor2 = new SparkMax(59, MotorType.kBrushless);

    // set up the config
    SparkFlexConfig hangmotor1Config = new SparkFlexConfig();
    // assign properties to motor
    hangmotor1Config
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);
        //.softLimit.reverseSoftLimit(0.17)
        //.reverseSoftLimitEnabled(true)
        
    hangmotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // set whether it will reset parameters when they are changed, and the persist
    // mode
    hangmotor1.configure(hangmotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // making motor4 follow motor3
    /*SparkMaxConfig hangmotor2Config = new SparkMaxConfig();
    hangmotor2Config.apply(hangmotor1Config);
    hangmotor2Config.follow(hangmotor1, true);
    hangmotor2.configure(hangmotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);*/

    // Quick and dirty way to enable position logging
    // The line is a no-op here but enables the desired packets
    hangmotor1.getEncoder().getPosition();
    hangmotor1.getAbsoluteEncoder().getPosition();
  }

  public Command joystickcontrol(Supplier<Double> joystickMove) {
    return new RepeatCommand(
        new InstantCommand(() -> hangmotor1.set(-joystickMove.get()), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }

  public Command Hangangle(double target, double range) {
    return new SparkPosition(hangmotor1, target, range, this).withName("goToPose");
  }

  public Command Hanging() {
    return Hangangle(180, 0.5);
  }

  public Command Hangset() {
    return Hangangle(0, 0.5);
  }

  public Command HangControl() {
    return new ConditionalCommand(
        Hanging(), // if yes
        Hangset(), // if no
        () -> {
          return hangmotor1.getAbsoluteEncoder().getPosition() < 180;
        }).withName("Hanging");
  }
}
