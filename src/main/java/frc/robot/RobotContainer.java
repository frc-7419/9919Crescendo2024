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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.BeamBreakSubsystem;

public class RobotContainer {
  
  // Joysticks, subsystems, and commands must all be private and final

  // Joysticks
  private final XboxController driver = new XboxController(Constants.OperatorConstants.kDriveControllerPort); //driver

  //Subsystems
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final BeamBreakSubsystem beambreak = new BeamBreakSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private final Command joystickShooter = new InstantCommand(() -> shooter.run(driver.getLeftY(), driver.getLeftY()), shooter);
  private final Command runShooter = new RunCommand(() -> shooter.run(11.5, 10.5), shooter); // change values later
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
    shooter.setDefaultCommand(joystickShooter);
  }
}