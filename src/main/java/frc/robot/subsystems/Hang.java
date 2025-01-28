// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


@Logged
public class Hang extends SubsystemBase {
//**creating the variables for the motors**
  private SparkMax motor5;
  private SparkMax motor6;
  

  public Hang() {
//assign cAn ID and Motor type
motor5 = new SparkMax(58,MotorType.kBrushless);
motor6 = new SparkMax(59,MotorType.kBrushless);

//set up the config
SparkMaxConfig motor5Config = new SparkMaxConfig();
//assign properties to motor
motor5Config
.smartCurrentLimit(30)
.idleMode(IdleMode.kBrake);
//set whether it will reset parameters when they are changed, and the persist mode
motor5.configure(motor5Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//making motor4 follow motor3
SparkMaxConfig motor6Config = new SparkMaxConfig();
motor6Config.apply(motor6Config);
motor6Config.follow(motor5, true);
motor6.configure(motor6Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
this.setDefaultCommand(
  new FunctionalCommand(
    ()->motor5.set(0),
     ()->{},
      (killed)->{},
       ()-> {return false;},
       this)

);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }
public Command hanging(){
return new InstantCommand(()->motor5.set(1),this);
  }
} 