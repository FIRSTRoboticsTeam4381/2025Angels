package frc.robot.subsystems;

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


@Logged
public class AlgaeIntake extends SubsystemBase 
{
    private SparkMax motor1;
    private SparkMax motor2;
    private DigitalInput sensor;
    
     public AlgaeIntake() 
        {
        motor1 = new SparkMax(50,MotorType.kBrushless);
        motor2 = new SparkMax(51,MotorType.kBrushless);
        sensor = new DigitalInput(0);

        
        SparkMaxConfig motor1Config = new SparkMaxConfig();
             motor1Config.smartCurrentLimit(25)
             .idleMode(IdleMode.kCoast);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    SparkMaxConfig motor2Config = new SparkMaxConfig();
    
        motor2Config.apply(motor1Config);
        motor2Config.follow(motor1, true);

    motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }  
    
    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        SmartDashboard.putData(this);
    }
        
        public Command Intake() 
                {
            return new SequentialCommandGroup(
                new InstantCommand(()-> motor1.set(-1), this),
                new WaitCommand(2),
                new InstantCommand(()-> motor1.set(0),this)
            ).withName("AlgaeIntake");  
                } 
         public Command Outtake() 
            {
            return new SequentialCommandGroup(
                new InstantCommand(()-> motor1.set(1), this),
                new WaitCommand(2),
                new InstantCommand(()-> motor1.set(0),this)
            ).withName("AlgaeOuttake");
            }
         public Command IntakeandOut() 
         {
            return new ConditionalCommand(Intake(), Outtake(), sensor::get).withName("AlgaeIntakeandOuttake");
         }
    
}