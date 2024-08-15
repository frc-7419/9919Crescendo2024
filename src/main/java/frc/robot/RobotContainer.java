package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveFieldCentric;
import frc.robot.commands.TranslateDistance;
import frc.robot.subsystems.amp.AmpSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.handoff.HandoffSubsystem;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeWristSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.BeamBreakSubsystem;


public class RobotContainer {
  
  // Joysticks, subsystems, and commands must all be private and final

  // Joysticks
  private final XboxController driver = new XboxController(Constants.OperatorConstants.kDriveControllerPort); //driver
  private final CommandXboxController operator = new CommandXboxController(1);


  //Subsystems
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  private final HandoffSubsystem handoff = new HandoffSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final BeamBreakSubsystem beambreak = new BeamBreakSubsystem();
  private final AmpSubsystem amp = new AmpSubsystem();
  
  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private final Command joystickShooter = new InstantCommand(() -> shooter.run(driver.getLeftY(), driver.getLeftY()), shooter);
  private final Command runShooter = new RunCommand(() -> shooter.run(11.5, 10.5), shooter); // change values later
  private final Command runIntake = new InstantCommand(() -> intake.run(driver.getLeftX(),driver.getLeftX()), intake);
  private final Command runIntakeAuton = new RunCommand(() -> intake.run(0.0,0.0), intake);
  private final Command raiseWrist = new RunCommand(() -> intakeWrist.goToPosition(0.0), intake);
  private final Command lowerWrist = new RunCommand(() -> intakeWrist.goToPosition(0.0), intake);
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  /**
   * Creates new RobotContainer and configures auton and buttons
   */
  public RobotContainer() { 
    configureButtonBindings();
    configureAutoSelector();
  }
  /**
   * This method is for configuring the button bindings on the joysticks
   */
  private void configureButtonBindings() {
    // Handoff
    // Run diverter clockwise. Ex: operator.y().whileTrue(new RunDiverter(diverter, 0.5));
    // Same for counter-clockwise
  }
  
  /**
   * This method is for configuring the auton chooser
   */
  private void configureAutoSelector() {
    SmartDashboard.putData("Auton", autonomousChooser);
  }
  /**
   * This will be the method which gets called to 
   * @return Auton command
   */
  public Command getAutonomousCommand() {
    return new TranslateDistance(driveBase, 1, 0);
  }
  /**
   * Sets default commands to be used for teleop
   */
  public void setDefaultCommands() {
    driveBase.setDefaultCommand(swerveDriveFieldCentric);
    shooter.setDefaultCommand(joystickShooter);
  }
}