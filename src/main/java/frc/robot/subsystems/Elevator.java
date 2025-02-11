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
import frc.robot.commands.SparkPosition;

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
             .inverted(true);

        SparkFlexConfig motorEL2Config = new SparkFlexConfig();

        motorEL2Config.apply(motorEL1Config);
        motorEL2Config.follow(motorEL1, false);
     
        motorEL2.configure(motorEL2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorEL1Config.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);

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
        new InstantCommand(() -> motorEL1.set(-joystickMove.get()),this)
      );
    }
    
    public Command goToPosition(double target, double range){
      return new SparkPosition(motorEL1, target, range, this).withName("goToPose");
    }
    
    
    public Command level4()
    {
      return goToPosition(400, 0.5);
    }
    public Command level3()
    {
      return goToPosition(300, 0.5);
    }
    public Command level2()
    {
      return goToPosition(200, 0.5);
    }
    public Command level1()
    {
      return goToPosition(100, 0.5);
    }
    public Command net()
    {
      return goToPosition(600, 0.5);
    }

}
