// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

@Logged
public class Coralintake extends SubsystemBase {
  // **creating the variables for the motors**

  private SparkFlex coralmotor1;
  private SparkFlex coralmotor2;

  // creating a sensor
  private SparkLimitSwitch coralsensor1;
  //private SparkLimitSwitch coralsensor2;

  // Creates a new Coralintake
  public Coralintake() {

    // assign cAn ID and Motor type
    coralmotor1 = new SparkFlex(55, MotorType.kBrushless);
    coralmotor2 = new SparkFlex(56, MotorType.kBrushless);

    // sets the can ID for a sensor
    coralsensor1 = coralmotor1.getForwardLimitSwitch();
    //coralsensor2 = coralmotor2.getForwardLimitSwitch();

    // set up the config
    SparkMaxConfig coralmotor1Config = new SparkMaxConfig();
    NamedCommands.registerCommand("coralinorout", coralInOrOut());
    NamedCommands.registerCommand("coralin", coralIn());
    NamedCommands.registerCommand("coralout", out().withTimeout(.3));
    NamedCommands.registerCommand("Mcoralintake", ManualCoarlIn().withTimeout(.3));
    NamedCommands.registerCommand("Mcoralout", ManualCoarlOut());
    NamedCommands.registerCommand("algaeInOrOut", algaeInOrOut());
    // assign properties to motor
    coralmotor1Config
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake).limitSwitch.forwardLimitSwitchEnabled(false);

    // set whether it will reset parameters when they are changed, and the persist
    // mode
    coralmotor1.configure(coralmotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // making motor4 follow motor3
    SparkMaxConfig coralmotor2Config = new SparkMaxConfig();
    coralmotor2Config.apply(coralmotor1Config);
    coralmotor2Config.follow(coralmotor1, true);// telling motor4 to be a follower of motor3

    coralmotor2.configure(coralmotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.setDefaultCommand(
        // sets default command

        new FunctionalCommand(
            // basic functional command

            () -> coralmotor1.set(0), // oninit
            () -> {
            }, // onexecute
            (killed) -> {
            }, // on end
            () -> {
              return false;
            }, // isfinished
            this)

    );

    // Quick and dirty way to enable position logging
    // The line is a no-op here but enables the desired packets
    coralmotor1.getEncoder().getVelocity();
    //coralmotor2.getEncoder().getVelocity();

    SmartDashboard.putData(ManualCoarlIn());
    SmartDashboard.putData(ManualCoarlOut());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
    SmartDashboard.putBoolean("Coral Int Running", Math.abs(coralmotor1.getAppliedOutput()) > 0);
  }

  public Command out() {// creating a sequential command group
    return new SequentialCommandGroup(
        // seting the motor speed to 1
        new InstantCommand(() -> coralmotor1.set(1), this),
        // checking to see if the sensor can see the coral
        new WaitUntilCommand(() -> !hascoral()),//!hascoral()),
        // wait time after throwing coral out
        new WaitCommand(1),
        // stopping the motor.
        new InstantCommand(() -> coralmotor1.set(0), this)
    // giving the action a name for logging/
    ).withName("Coral Outaking");
  };

  public Command coralIn() {
    return new SequentialCommandGroup(
        // this command will run until the sensor sees the coral
        new InstantCommand(() -> coralmotor1.set(-1), this),
        RobotContainer.getRobot().vibrateSpecialistWhile(RumbleType.kRightRumble, 0.5,
            new WaitUntilCommand(() -> hascoral())),
        RobotContainer.getRobot().vibrateDriverForTime(RumbleType.kBothRumble, 0.8, 0.5),
        new WaitCommand(0.5),
        new InstantCommand(() -> coralmotor1.set(0), this)//,
        //new WaitCommand(1),
        //new InstantCommand(() -> coralmotor1.set(-1), this),
        //new WaitCommand(0.75),
        //new InstantCommand(() -> coralmotor1.set(0), this)
        )
        .withName("Coral Intaking");
  }

  public Command algaeIn() {
    return new SequentialCommandGroup(
        // this command will run until the sensor sees the coral
        new InstantCommand(() -> coralmotor1.set(-1), this),
        RobotContainer.getRobot().vibrateSpecialistWhile(RumbleType.kRightRumble, 0.5,
            new WaitUntilCommand(() -> hascoral())),//hascoral())),
        RobotContainer.getRobot().vibrateDriverForTime(RumbleType.kBothRumble, 0.8, 0.5),
        new InstantCommand(() -> coralmotor1.set(0), this)).withName("Coral Intaking");
  }

  public Command coralInOrOut() {
    return new ConditionalCommand(
        out(), // this will run when the sensor sees something
        coralIn(), // this will run when the sensor doesn't see anything
        this::hascoral).withName("Coral in or out is running"); // this tells it which sensor to use
  }

  public Command algaeInOrOut() {
    return new ConditionalCommand(
        out(), // this will run when the sensor sees something
        algaeIn(), // this will run when the sensor doesn't see anything
        this::hascoral).withName("Algae in or out is running"); // this tells it which sensor to use
  }

  public boolean hascoral() {
    return coralsensor1.isPressed(); //|| coralsensor2.isPressed();
  }
   public Command ManualCoarlIn() 
  {
    return new InstantCommand(() -> coralmotor1.set(-.3), this).repeatedly();
  }
  public Command ManualCoarlOut() 
  {
    return new InstantCommand(() -> coralmotor1.set(.5), this).repeatedly();
  }

}
