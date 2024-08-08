package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveFieldCentric;
import frc.robot.commands.TranslateDistance;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeWristSubsystem;


public class RobotContainer {
  
  // Joysticks, subsystems, and commands must all be private and final

  // Joysticks
  private final XboxController driver = new XboxController(0); //driver

  //Subsystems
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private final Command runIntake = new InstantCommand(() -> intake.run(driver.getLeftX(),driver.getLeftX()), intake);
  private final Command runIntakeAuton = new RunCommand(() -> intake.run(0.0,0.0), intake);
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
  }
}