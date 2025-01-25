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
public class Coralintake extends SubsystemBase {
//**creating the variables for the motors**
  private SparkMax motor3;
  private SparkMax motor4;
  //creating a sensor
  private DigitalInput coralsensor;
  // Creates a new Coralintake
  public Coralintake() {
//assign cAn ID and Motor type
motor3 = new SparkMax(55,MotorType.kBrushless);
motor4 = new SparkMax(56,MotorType.kBrushless);
//sets the can ID for a sensor
coralsensor = new DigitalInput(57);
coralsensor.get();

//set up the config
SparkMaxConfig motor3Config = new SparkMaxConfig();
//assign properties to motor
motor3Config
.smartCurrentLimit(30)
.idleMode(IdleMode.kBrake);
//set whether it will reset parameters when they are changed, and the persist mode
motor3.configure(motor3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//making motor4 follow motor3
SparkMaxConfig motor4Config = new SparkMaxConfig();
motor4Config.apply(motor4Config);
motor4Config.follow(motor3//telling motor4 to be a follower of motor3
);
motor4.configure(motor4Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
this.setDefaultCommand(
  //sets default command
  new FunctionalCommand(
    //basic functional command
    ()->motor3.set(0), //oninit
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
    new InstantCommand(()-> motor3.set(1)),
    //checking to see if the sensor can see the coral
    new WaitUntilCommand(()->!coralsensor.get()),
    //wait time after throwing coral out
    new WaitUntilCommand(1.5),
    //stopping the motor.
    new InstantCommand(()->motor3.set(0))
    //giving the action a name for logging/
  ).withName("Coral Outaking");
  };

  public Command Coralin()
  {
    return new SequentialCommandGroup(
      //this command will run until the sensor sees the coral
      new InstantCommand(()-> motor3.set(-1))
      .until(coralsensor::get),
      new WaitUntilCommand(0.5),
      new InstantCommand(()->motor3.set(0))
      ).withName("Coral Intaking");
  }

  public Command Coralinorout()
  {
    return new ConditionalCommand(
      CoralOut(), // this will run when the sensor sees something
       Coralin(), //this will run when the sensor doesn't see anything
        coralsensor::get); //this tells it which sensor to use
  }
}







