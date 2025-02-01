package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

@Logged
public class AlgaeIntake extends SubsystemBase {
    private SparkMax algaemotor1;
    private SparkMax algaemotor2;
    private DigitalInput algaesensor;

    public AlgaeIntake() {
        algaemotor1 = new SparkMax(50, MotorType.kBrushless);
        algaemotor2 = new SparkMax(51, MotorType.kBrushless);
        algaesensor = new DigitalInput(0);

        SparkMaxConfig algaemotor1Config = new SparkMaxConfig();
        algaemotor1Config.smartCurrentLimit(10)
                .idleMode(IdleMode.kCoast);

        algaemotor1.configure(algaemotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig algaemotor2Config = new SparkMaxConfig();

        algaemotor2Config.apply(algaemotor1Config);
        algaemotor2Config.follow(algaemotor1, true);

        algaemotor2.configure(algaemotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        NamedCommands.registerCommand("InAndOut", IntakeandOut());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData(this);
    }

    public Command Intake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> algaemotor1.set(-1), this),
                new WaitUntilCommand(() -> !algaesensor.get()),
                new WaitCommand(1.5),
                new InstantCommand(() -> algaemotor1.set(0), this)).withName("AlgaeIntake");
    }

    public Command Outtake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> algaemotor1.set(1), this),
                new WaitUntilCommand(() -> algaesensor.get()),
                new WaitCommand(1.5),
                new InstantCommand(() -> algaemotor1.set(0), this)).withName("AlgaeOuttake");
    }

    public Command IntakeandOut() {
        return new ConditionalCommand(Intake(), Outtake(), algaesensor::get).withName("AlgaeIntakeandOuttake");
    }

}