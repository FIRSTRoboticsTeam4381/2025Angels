// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This subsystem uses a Spark Max in !brushed! mode as a simplified LED driver.
 * Since the LEDs are diodes, forward power (+12v) can drive one strand, and reverse (-12v)
 * can drive another strand wired the opposite way.
 */
@Logged
public class ChuteLEDs extends SubsystemBase {
  
  public SparkMax ledController;
  
  /** Creates a new ChuteLEDs. */
  public ChuteLEDs() {
    ledController = new SparkMax(5, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setLeftChute()
  {
    return new InstantCommand(() -> ledController.set(1), this)
      .ignoringDisable(true)
      .withName("Left");
  }

  public Command setRightChute()
  {
    return new InstantCommand(() -> ledController.set(-1), this)
      .ignoringDisable(true)
      .withName("Right");
  }
}
