package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

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
public class AlgaeIntake extends SubsystemBase {
    private SparkMax algaemotor1;
    //private SparkMax algaemotor2;
    private SparkLimitSwitch algaesensor;
    //private SparkMax algaemotor3;
    private SparkFlex groundIntake;

    public AlgaeIntake() {
        algaemotor1 = new SparkMax(54, MotorType.kBrushless);
        //algaemotor2 = new SparkMax(51, MotorType.kBrushless);
        algaesensor = algaemotor1.getForwardLimitSwitch();
        //algaemotor3 = new SparkMax(50,MotorType.kBrushless);
        groundIntake = new SparkFlex(53, MotorType.kBrushless);

        SparkMaxConfig algaemotor1Config = new SparkMaxConfig();
        algaemotor1Config.smartCurrentLimit(15)
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .limitSwitch.forwardLimitSwitchEnabled(false);

        algaemotor1.configure(algaemotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //SparkMaxConfig algaemotor2Config = new SparkMaxConfig();
        //SparkMaxConfig algaemotor3Config = new SparkMaxConfig();

        /*algaemotor2Config.apply(algaemotor1Config);
        algaemotor2Config.follow(algaemotor1, true);
        algaemotor3Config.apply(algaemotor1Config);
        algaemotor3Config.follow(algaemotor1, false);*/

        //algaemotor2.configure(algaemotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //algaemotor3.configure(algaemotor3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkFlexConfig groundIntakeConfig = new SparkFlexConfig();
        groundIntakeConfig.smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake)
        .closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
     
        groundIntakeConfig.closedLoop.p(3.5);
        groundIntake.configure(groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NamedCommands.registerCommand("InAndOut", IntakeandOut());

        

        this.setDefaultCommand(
        // sets default command

        new FunctionalCommand(
            // basic functional command

            () -> {algaemotor1.set(0);
           }, // oninit
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
        algaemotor1.getEncoder().getVelocity();
        //algaemotor2.getEncoder().getVelocity();
        //algaemotor3.getEncoder().getVelocity();
        groundIntake.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("Algae Int Running", Math.abs(algaemotor1.getAppliedOutput()) > 0);
    }

    public Command Intake() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> groundIntake.getClosedLoopController().setReference(.66, ControlType.kPosition)),
                new InstantCommand(() -> algaemotor1.set(-1), this),
               RobotContainer.getRobot().vibrateSpecialistWhile(RumbleType.kLeftRumble,0.5,
                new WaitUntilCommand(() -> hasalgae())),
                RobotContainer.getRobot().vibrateDriverForTime(RumbleType.kLeftRumble,0.5,0.5),
                new WaitCommand(0.25),
                new InstantCommand(() -> algaemotor1.set(0), this))
                .handleInterrupt(() -> groundIntake.getClosedLoopController().setReference(.36, ControlType.kPosition))
                .withName("AlgaeIntake");
    }

    public Command Outtake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> algaemotor1.set(1), this),
                new WaitUntilCommand(() -> !hasalgae()),
                new WaitCommand(2.2),
                new InstantCommand(() -> algaemotor1.set(0), this),
                new InstantCommand (() ->  groundIntake.getClosedLoopController().setReference(.36, ControlType.kPosition)))
                .withName("AlgaeOuttake");

    }

    public Command IntakeandOut() {
        return new ConditionalCommand(Outtake(), Intake(), this::hasalgae).withName("AlgaeIntakeandOuttake");
    }

    public boolean hasalgae()
    {
        return algaesensor.isPressed();
    }

}