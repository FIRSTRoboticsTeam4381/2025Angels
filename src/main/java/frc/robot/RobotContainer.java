// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AdvancedCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.SnaptoPose;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ChuteLEDs;
import frc.robot.subsystems.Coralintake;
import frc.robot.subsystems.DriverCam;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();

  // Subsystems
  public final Swerve swerve;
  public final Coralintake coralintake;
 // public final AlgaeIntake algaeintake;
  public final Elevator elevator;
  public final Hang hang;
  public final Pivot pivot;
  

  public final PhotonCam camA = new PhotonCam("FR", new Transform3d(
    new Translation3d(Units.inchesToMeters(9.6), Units.inchesToMeters(-10.43),  Units.inchesToMeters(14.125+0.17)), 
    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-3.06),Units.degreesToRadians(17.11) )));
  public final PhotonCam camB = new PhotonCam("FL", new Transform3d(
    new Translation3d(Units.inchesToMeters(9.6), Units.inchesToMeters(10.43),  Units.inchesToMeters(14.125+0.17)), 
    new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(-3.06),Units.degreesToRadians(-17.11))));

  public final PhotonCam camC = new PhotonCam("BL", new Transform3d(
    new Translation3d(Units.inchesToMeters(0.13), Units.inchesToMeters(9.44),  Units.inchesToMeters(12.76)),
    new Rotation3d(0,Units.degreesToRadians(-16.5), Units.degreesToRadians(47.52+180))));
  public final PhotonCam camD = new PhotonCam("BR", new Transform3d(
    new Translation3d(Units.inchesToMeters(0.13), Units.inchesToMeters(-9.44),  Units.inchesToMeters(12.76)),
    new Rotation3d(0,Units.degreesToRadians(-16.5),  Units.degreesToRadians(-47.52-180))) );

    public DriverCam hangCam = new DriverCam("HangCamera");
    public ChuteLEDs chuteLEDs;

  public final AdvancedCommands advancedCommands;
  // Constructor: set up the robot! 
  public RobotContainer() {
    robotReference = this;
    
    swerve = new Swerve();
    coralintake = new Coralintake();
    //algaeintake  = new AlgaeIntake();
    elevator = new Elevator();
    hang = new Hang();
    pivot = new Pivot();
    advancedCommands = new AdvancedCommands(this);

    // Set default commands here

    chuteLEDs = new ChuteLEDs();



    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("Test", Autos.testAuto());
    //autoChooser.addOption("Bottom to E to D", Autos.BottomtoEtoD());
    autoChooser.addOption("Bottom to F to E", Autos.BottomtoFtoE());
    autoChooser.addOption("Top To I To J", Autos.TopToIToJ());
    //autoChooser.addOption("Top To I To K", Autos.TopToIToK());
    //autoChooser.addOption("Top To I To L", Autos.TopToIToL());
    //autoChooser.addOption("Bottom to BB to MB", Autos.BottomtoBBtoMB());
    //autoChooser.addOption("CustomAutoTop", Autos.JtoTCoarlstation());
    autoChooser.addOption("CustomAutoBottom", Autos.EtoBCoralStation());
    SmartDashboard.putString("Choose Reef", "");
    SmartDashboard.putString("ReefSelectTop", "");
    

    // Add auto controls to the dashboard
    SmartDashboard.putData("Choose Auto:", autoChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    autoChooser.onChange((listener) -> listener.showPreview());
    SmartDashboard.putNumber("Start Delay",0);

    
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {
    driver.back()
      .onTrue(new InstantCommand(() -> swerve.zeroYaw()).ignoringDisable(true));
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
          driver::getLeftY,
          driver::getLeftX,
          driver::getRightX,
             true, driver.leftBumper()::getAsBoolean));

specialist.rightBumper().toggleOnTrue(coralintake.Coralinorout());
//specialist.leftBumper().toggleOnTrue(algaeintake.IntakeandOut());
elevator.setDefaultCommand(elevator.joystickcontrol(interpolateJoystick(specialist::getLeftY, Constants.stickDeadband)));
pivot.setDefaultCommand(pivot.joystickcontrol(interpolateJoystick(specialist::getRightY, Constants.stickDeadband)));
hang.setDefaultCommand(hang.joystickcontrol(() -> specialist.getLeftTriggerAxis() - specialist.getRightTriggerAxis()));
//specialist.x().onTrue(hang.HangControl());
specialist.povUp().onTrue(advancedCommands.l4().andThen(advancedCommands.hold()));
specialist.povRight().onTrue(advancedCommands.l3().andThen(advancedCommands.hold()));
specialist.povDown().onTrue(advancedCommands.l2().andThen(advancedCommands.hold()));
specialist.povLeft().onTrue(advancedCommands.l1().andThen(advancedCommands.hold()));
driver.rightBumper().whileTrue(new SnaptoPose(swerve));
driver.leftBumper().whileTrue(swerve.setCoast());
specialist.y().onTrue(advancedCommands.NetAlgae());
specialist.x().onTrue(pivot.algaepickup());
specialist.a().onTrue(pivot.Proccesor());
specialist.b().onTrue(advancedCommands.AlgaeReef());
//specialist.a().onTrue(coralintake.ManualCoarlIn());
//specialist.y().onTrue(coralintake.ManualCoarlOut());

specialist.axisMagnitudeGreaterThan(1, Constants.stickDeadband).onTrue(elevator.getDefaultCommand());
specialist.axisMagnitudeGreaterThan(5, Constants.stickDeadband).onTrue(pivot.getDefaultCommand());


specialist.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));


driver.leftTrigger(0.5).onTrue(chuteLEDs.setLeftChute());
driver.rightTrigger(0.5).onTrue(chuteLEDs.setRightChute());



  }

  public Command getAutonomousCommand() {
    Autos.pickReef();
    double startDelay=SmartDashboard.getNumber("Start Delay", 0);
    return new SequentialCommandGroup( 
    new WaitCommand(startDelay), 
    new ScheduleCommand(autoChooser.getSelected().auto)); 
  }


  /**
   * Smooths joystic input for easier precice control without sacrificing full power.
   * @param in Input from joystic
   * @param deadzone Joystick deadzone
   * @return Transformed output
   */
  public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
      return () -> interpolateNow(in.get(), deadzone);
  }

  public static double interpolateNow(double in, double deadzone)
  {
      if(Math.abs(in) < deadzone)
          return 0.0;
      else if (in>0)
          return Math.pow((in - deadzone)*(1.0/(1.0-deadzone)), 3);
      else 
          return -Math.pow((-in - deadzone)*(1.0/(1.0-deadzone)), 3);
  }

  /*public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
      return () -> interpolateNow(in.get(), deadzone);
  }

  public static double interpolateNow(double in, double deadzone)
  {
    if(Math.abs(in) < deadzone)
      return 0.0;
    else
      return in;
  }
*/
    
  // Static reference to the robot class
  // Previously we used static subsystems, but this appears to break things in 2025
  // Use getRobot() to get the robot object
  private static RobotContainer robotReference;

  /**
   * Get a reference to the RobotContainer object in use
   * @return the active RobotContainer object
   */
  public static RobotContainer getRobot()
  {
    return robotReference;
  }

  /**
   * Set rumble for the specialist controller
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @return Instant command to set rumble strength
   */
  public Command vibrateSpecialist(RumbleType rumbleside, double rumblestrength )
  {
    return new InstantCommand(() -> specialist.setRumble(rumbleside, rumblestrength));
  }

  /**
   * Set rumble for the driver controller
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @return Instant command to set rumble strength
   */
  public Command vibrateDriver(RumbleType rumbleside, double rumblestrength )
  {
    return new InstantCommand(() -> driver.setRumble(rumbleside, rumblestrength));
  }

  /**
   * Rumble a controller while a command is running, stopping when the command finishes
   * @param controller Controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateWhile(CommandXboxController controller, RumbleType rumbleside, double rumblestrength, Command c)
  {
    return new ParallelRaceGroup(
      c,
      new FunctionalCommand(() -> controller.setRumble(rumbleside, rumblestrength),
      () -> controller.setRumble(rumbleside, rumblestrength),
      (interrupted) -> controller.setRumble(rumbleside, 0),
      () -> {return false;})
    );
  }

  /**
   * Rumble specials controller while a command is running, stopping when the command finishes
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateSpecialistWhile(RumbleType rumbleside, double rumblestrength, Command c)
  {
    return vibrateWhile(specialist, rumbleside, rumblestrength, c);
  }

  /**
   * Rumble driver controller while a command is running, stopping when the command finishes
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateDriverWhile(RumbleType rumbleside, double rumblestrength, Command c)
  {
    return vibrateWhile(driver, rumbleside, rumblestrength, c);
  }

  /**
   * Set specialist controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateSpecialistForTime(RumbleType rumbleside, double rumblestrength, double time)
  {
    return vibrateForTime(specialist, rumbleside, rumblestrength, time);
  }

  /**
   * Set driver controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateDriverForTime(RumbleType rumbleside, double rumblestrength, double time)
  {
    return vibrateForTime(driver, rumbleside, rumblestrength, time);
  }

  /**
   * Set a controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param controller controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateForTime(CommandXboxController controller,RumbleType rumbleside, double rumblestrength, double time)
  {
    return new ScheduleCommand(vibrateWhile(controller, rumbleside, rumblestrength, new WaitCommand(time)));
  }
}
