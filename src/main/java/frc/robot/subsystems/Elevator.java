// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPositionProfiled;

@Logged
public class Elevator extends SubsystemBase {
 
  private SparkFlex motorEL1;
  private SparkFlex motorEL2;
 
  /** Creates a new Elevator. */
  public Elevator() {
        motorEL1 = new SparkFlex(60,MotorType.kBrushless);
        motorEL2 = new SparkFlex(61,MotorType.kBrushless);

        SparkFlexConfig motorEL1Config = new SparkFlexConfig();
             motorEL1Config.smartCurrentLimit(60)
             .idleMode(IdleMode.kBrake)
             .inverted(true)
             .softLimit.forwardSoftLimit(72)
             .forwardSoftLimitEnabled(true);
             motorEL1Config.closedLoop.p(.3);
             motorEL1Config.closedLoop.d(3);
             motorEL1Config.closedLoop.maxMotion.maxAcceleration(9000).maxVelocity(5000);


             //motorEL1Config.closedLoopRampRate(.5);


        SparkFlexConfig motorEL2Config = new SparkFlexConfig();

        motorEL2Config.apply(motorEL1Config);
        motorEL2Config.follow(motorEL1, false);
     
        motorEL2.configure(motorEL2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorEL1Config.limitSwitch.forwardLimitSwitchEnabled(true).reverseLimitSwitchEnabled(true);

        motorEL1.configure(motorEL1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Quick and dirty way to enable position logging
        // The line is a no-op here but enables the desired packets
        motorEL1.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }
  
    // This method will be called once per scheduler run
   
    public Command joystickcontrol(Supplier<Double> joystickMove)
    {
      return new RepeatCommand(
        new InstantCommand(() -> motorEL1.set(-joystickMove.get()*.3),this)
      );
    }
    
    public Command goToPosition(double target, double range){
      return new SparkPositionProfiled(motorEL1, target, range, this).withName("goToPose");
    }
    
    
    public Command level4()
    {
      return goToPosition(69.3, 0.2).withName("l4");
    }
    public Command level3()
    {
      return goToPosition(36.6, 0.2).withName("l3");
    }
    public Command level2()
    {
      return goToPosition(15.52, 0.2).withName("l2");
    }
    public Command level1()
    {
      return goToPosition(0.78, 0.5).withName("l1");
    }
    public Command net()
    {
      return goToPosition(71.5, 0.5);
    }
    public Command Algaereef()
    {
      return goToPosition(50.34, 0.5);
    }
    public Command lollypopalgae()
    {
      return goToPosition(0.26, 0.5);
    }
    public Command lowalgae()
    {
      return goToPosition(34, 0.5);
    }
    public Command algael2()
    {
      return goToPosition(31, 0.5);
    }
    public Command algael3()
    {
      return goToPosition(53, 0.5);
    }
    public Command corall1()
    {
      return goToPosition(38, 0.5);
    }
}
