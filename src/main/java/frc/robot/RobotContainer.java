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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AdvancedCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.SnapToAngle;
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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  //public final CommandXboxController specialist = new CommandXboxController(1);
  public CommandGenericHID specialGenericHID = new CommandGenericHID(1);
  public CommandGenericHID special2GenericHID = new CommandGenericHID(2);
  
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
    new Translation3d(Units.inchesToMeters(0.07), Units.inchesToMeters(9.41),  Units.inchesToMeters(12.85)),
    new Rotation3d(0,Units.degreesToRadians(-16.91), Units.degreesToRadians(37.4+180))));
  public final PhotonCam camD = new PhotonCam("BR", new Transform3d(
    new Translation3d(Units.inchesToMeters(0.07), Units.inchesToMeters(-9.41),  Units.inchesToMeters(12.85)),
    new Rotation3d(0,Units.degreesToRadians(-16.91),  Units.degreesToRadians(-37.4-180))) );

    public DriverCam hangCam = new DriverCam("HangCamera");
   // public ChuteLEDs chuteLEDs;

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

    //chuteLEDs = new ChuteLEDs();



    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    //autoChooser.addOption("Test", Autos.testAuto());
    //autoChooser.addOption("Bottom to E to D", Autos.BottomtoEtoD());
    autoChooser.addOption("Blue Right F-E", Autos.BlueRightFToE());
    //autoChooser.addOption("Red Right F-E", Autos.RedRightFToE());
    autoChooser.addOption("Left I-J", Autos.TopToIToJ());
    //autoChooser.addOption("Middle auto", Autos.Middleauto());
    autoChooser.addOption("Middle Ball", Autos.middleball());
    autoChooser.addOption("Blue Middle Ball", Autos.bluemiddleball());
    autoChooser.addOption("Extra Blue Middle Ball", Autos.extrabluemiddleball());
    autoChooser.addOption("Extra Middle Ball", Autos.extramiddleball());
    //autoChooser.addOption("Top To I To K", Autos.TopToIToK());
    //autoChooser.addOption("Top To I To L", Autos.TopToIToL());
    //autoChooser.addOption("Bottom to BB to MB", Autos.BottomtoBBtoMB());
    //autoChooser.addOption("CustomAutoTop", Autos.JtoTCoarlstation());
    //autoChooser.addOption("CustomAutoBottom", Autos.EtoBCoralStation());
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
             true, () -> driver.getRightTriggerAxis() > 0.5));


  //if(special2GenericHID.button(12).whileTrue())
 // {
 //   return new InstantCommand(Coralintake(()-> coralIn()));
  //}
  
  special2GenericHID.button(12).and(new Trigger(coralintake::hascoral)).whileTrue(coralintake.ManualCoarlIn());
  special2GenericHID.button(12).and(new Trigger(coralintake::hascoral).negate()).onTrue(coralintake.coralIn());
  special2GenericHID.button(7).and(new Trigger(coralintake::hascoral)).whileTrue(coralintake.ManualAlgaeIn());
  special2GenericHID.button(7).and(new Trigger(coralintake::hascoral).negate()).onTrue(coralintake.algaeIn());
  specialGenericHID.button(10).and(new Trigger(coralintake::hascoral)).onTrue(coralintake.out());
  specialGenericHID.button(10).and(new Trigger(coralintake::hascoral).negate()).whileTrue(coralintake.ManualOut());


//.and(
   // special2GenericHID.start().negate()
 // ).toggleOnTrue(coralintake.coralInOrOut());
//specialist.leftBumper().and(
  //specialist.start().negate()
//.toggleOnTrue(coralintake.algaeInOrOut());
elevator.setDefaultCommand(elevator.joystickcontrol(interpolateJoystick(stretchJoystick(()-> special2GenericHID.getRawAxis(1), -1.0, 1.0), Constants.stickDeadband)));
pivot.setDefaultCommand(pivot.joystickcontrol(interpolateJoystick(stretchJoystick(()-> specialGenericHID.getRawAxis(0), -1.00, 1.00), Constants.stickDeadband)));
hang.setDefaultCommand(hang.joystickcontrol(interpolateJoystick(stretchJoystick(() -> specialGenericHID.getRawAxis(1), -1.00, 1.00), Constants.stickDeadband )));
//specialist.x().onTrue(hang.HangControl());
specialGenericHID.button(4).and (special2GenericHID.button(1)).onTrue(advancedCommands.l4Teleop().andThen(advancedCommands.hold()));
specialGenericHID.button(3).and (special2GenericHID.button(1)).onTrue(advancedCommands.l3Teleop().andThen(advancedCommands.hold()));
specialGenericHID.button(2).and(special2GenericHID.button(1)).onTrue(advancedCommands.l2Teleop().andThen(advancedCommands.hold()));
//specialist.povDown().and(specialist.start()).onTrue(elevator.level2().andThen(advancedCommands.hold()));
specialGenericHID.button(6).onTrue(advancedCommands.parallelHome().andThen(advancedCommands.hold()));
driver.rightBumper().whileTrue(new SnaptoPose(swerve));
driver.leftBumper().whileTrue(swerve.setCoast());
specialGenericHID.button(12).onTrue(advancedCommands.NetAlgae().andThen(advancedCommands.hold()));
specialGenericHID.button(8).onTrue(advancedCommands.lollypop().andThen(advancedCommands.hold()));
specialGenericHID.button(11).onTrue(advancedCommands.processor().andThen(advancedCommands.hold()));
special2GenericHID.button(8).onTrue(advancedCommands.algaeGround().andThen(advancedCommands.hold()));
special2GenericHID.button(6).onTrue(advancedCommands.algaeL2().andThen(advancedCommands.hold()));
specialGenericHID.button(7).onTrue(advancedCommands.algaeL3().andThen(advancedCommands.hold()));
//specialist.b().onTrue(advancedCommands.AlgaeReef());

driver.x().whileTrue(swerve.brake());
driver.start().toggleOnTrue(new TeleopSwerve(swerve, 
driver::getLeftY,
driver::getLeftX,
driver::getRightX,
   true, () -> false , false));

special2GenericHID.axisMagnitudeGreaterThan(1, Constants.stickDeadband).onTrue(elevator.getDefaultCommand());
specialGenericHID.axisMagnitudeGreaterThan(0, Constants.stickDeadband).onTrue(pivot.getDefaultCommand());


special2GenericHID.button(11).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
specialGenericHID.button(1).and (special2GenericHID.button(1)).onTrue(advancedCommands.coralL1().andThen(advancedCommands.hold()));


//driver.leftTrigger(0.5).onTrue(chuteLEDs.setLeftChute());
//driver.rightTrigger(0.5).onTrue(chuteLEDs.setRightChute());


//specialist.leftBumper().and(specialist.start()).whileTrue(coralintake.ManualCoarlIn());
//specialist.rightBumper().and(specialist.start()).whileTrue(coralintake.ManualCoarlOut());

driver.leftTrigger(0.2).whileTrue(new SnapToAngle(swerve));

  }

  public Command getAutonomousCommand() {
    Autos.pickReef();
    double startDelay=SmartDashboard.getNumber("Start Delay", 0);
    return new SequentialCommandGroup( 
    new WaitCommand(startDelay), 
    new ScheduleCommand(autoChooser.getSelected().auto)); 
  }


  /**
   * This function fixes joysticks where an axis doesn't quite reach
   * 1.00 or -1.00 (but correctly rests at 0) by stretching the positive
   * and/or negative range of the joystick based on the min/max value it reads.
   * @param in Supplier for raw joystick value
   * @param min Minimum measured joystick value. Must be between -1.00 and 0.
   * @param max Maximum measured joystick value. Must be between 0 and 1.00.
   * @return Supplier which resolves to a "fixed" joystick value that reaches
   * from -1.00 to 1.00.
   */
  public static Supplier<Double> stretchJoystick(Supplier<Double> in, double min, double max)
  {
    return () -> {
      double x = in.get();
      if(x>0)
        // Stretch positive joystick value
        return x * (1d / max);
      else if(x < 0)
        // Stretch negative joystick value
        return x * (1d / -min);
      else
        return 0d;
    };
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
    return Commands.none();
    //return new InstantCommand(() -> specialist.setRumble(rumbleside, rumblestrength));
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
    return Commands.none();
  //  return vibrateWhile(specialist, rumbleside, rumblestrength, c);
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
    return Commands.none();
    //return vibrateForTime(specialist, rumbleside, rumblestrength, time);
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
