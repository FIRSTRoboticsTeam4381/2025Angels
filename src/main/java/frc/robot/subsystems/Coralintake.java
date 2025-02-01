// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
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
//**creating the variables for the motors**

  private SparkMax coralmotor1;
  private SparkMax coralmotor2;

  //creating a sensor
  private DigitalInput coralsensor;

  // Creates a new Coralintake
  public Coralintake() {

  //assign cAn ID and Motor type
  coralmotor1 = new SparkMax(55,MotorType.kBrushless);
  coralmotor2 = new SparkMax(56,MotorType.kBrushless);

  //sets the can ID for a sensor
  coralsensor = new DigitalInput(1);
  coralsensor.get();

  //set up the config
  SparkMaxConfig coralmotor1Config = new SparkMaxConfig();
  NamedCommands.registerCommand("coralinorout", Coralinorout());
  //assign properties to motor
  coralmotor1Config
  .smartCurrentLimit(10)
  .idleMode(IdleMode.kBrake);

  //set whether it will reset parameters when they are changed, and the persist mode
  coralmotor1.configure(coralmotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //making motor4 follow motor3
  SparkMaxConfig coralmotor2Config = new SparkMaxConfig();
  coralmotor2Config.apply(coralmotor2Config);
  coralmotor2Config.follow(coralmotor1//telling motor4 to be a follower of motor3
);

coralmotor2.configure(coralmotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

this.setDefaultCommand(
  //sets default command

  new FunctionalCommand(
    //basic functional command

    ()->coralmotor1.set(0), //oninit
    ()->{}, //onexecute
    (killed)->{}, //on end
    ()->{return false;}, //isfinished
    this)
    
);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }

  public Command CoralOut()
{//creating a sequential command group
  return new SequentialCommandGroup(
    //seting the motor speed to 1
    new InstantCommand(()-> coralmotor1.set(1),this),
    //checking to see if the sensor can see the coral
    new WaitUntilCommand(()->!coralsensor.get()),
    //wait time after throwing coral out
    new WaitCommand(1.5),
    //stopping the motor.
    new InstantCommand(()->coralmotor1.set(0),this)
    //giving the action a name for logging/
  ).withName("Coral Outaking");
  };

  public Command Coralin()
  {
    return new SequentialCommandGroup(
      //this command will run until the sensor sees the coral
      RobotContainer.getRobot().vibrateSpecialist(RumbleType.kRightRumble, .5),
      new InstantCommand(()-> coralmotor1.set(-1),this),
      new WaitUntilCommand(()->coralsensor.get()),
      RobotContainer.getRobot().vibrateSpecialist(RumbleType.kRightRumble, 0),
      //makes controller rumble on the right side
      new WaitCommand(0.5),
      new InstantCommand(()->coralmotor1.set(0),this)
      ).withName("Coral Intaking");
  }

  public Command Coralinorout()
  {
    return new ConditionalCommand(
      CoralOut(), // this will run when the sensor sees something
       Coralin(), //this will run when the sensor doesn't see anything
        coralsensor::get).withName("Coral in or out is running"); //this tells it which sensor to use
  }
}







