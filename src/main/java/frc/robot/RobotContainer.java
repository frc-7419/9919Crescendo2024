package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DetectTag;
import frc.robot.commands.SwerveDriveFieldCentric;
import frc.robot.commands.TranslateDistance;
import frc.robot.subsystems.LimelightRangeChecker;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeWristSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.BeamBreakSubsystem;


public class RobotContainer {

    // JOYSTICKS, SUBSYSTEMS, AND COMMANDS MUST ALL BE PRIVATE AND FINAL

    /*
     * Joystick
     *  - The objects in this catagory are joysticks, they should be either a CommandXboxController, XboxController, or a Joystick
     *  - The joystick objects handle handle all input from joysticks which are plugged in to the Driver Station
     *  - The port of the joystick is selected in the Driver Station, generally we have the drive controller in port 0 while the operator controller uses port 1
     */
    private final XboxController driver = new XboxController(Constants.ControllerConstants.kDriveControllerPort);
    private final CommandXboxController operator = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);


    /*
     * Subsystems
     *  - This catagory is for all subsystems which the robot has.
     *  - Subsystems should have methods which do low-level things such as running a motor, there shouldn't be complicated algorithms
     */
    // private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();
    // private final HandoffSubsystem handoff = new HandoffSubsystem();
    // private final IntakeSubsystem intake = new IntakeSubsystem();
    // private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();
    // private final ShooterSubsystem shooter = new ShooterSubsystem();
    // private final BeamBreakSubsystem beambreak = new BeamBreakSubsystem();
    private final LimelightRangeChecker rangeChecker = new LimelightRangeChecker();

    /*
     * Commands
     *  - This catagory is for commands, the commands can either be lambda instant or run commands, or they can be classes
     */
    // private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
    private final DetectTag detectTag = new DetectTag(rangeChecker);
    // private final Command joystickShooter = new InstantCommand(() -> shooter.run(driver.getLeftY(), driver.getLeftY()), shooter);
    // private final Command runShooter = new RunCommand(() -> shooter.run(11.5, 10.5), shooter); // change values later
    // private final Command runIntake = new InstantCommand(() -> intake.run(driver.getLeftX(), driver.getLeftX()), intake);
    // private final Command runIntakeAuton = new RunCommand(() -> intake.run(0.0, 0.0), intake);
    // private final Command raiseWrist = new RunCommand(() -> intakeWrist.goToPosition(0.0), intake);
    // private final Command lowerWrist = new RunCommand(() -> intakeWrist.goToPosition(0.0), intake);
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
     * This method gets callled in Robot to schedule the auton command, so whatever commond this method returns will be the one running during the autonomous period
     * @return Auton command
     */
    public Command getAutonomousCommand() {
        return null;//new TranslateDistance(driveBase, 1, 0);
    }

    /**
     * Sets default commands to be used for teleop
     */
    public void setDefaultCommands() {
        //driveBase.setDefaultCommand(swerveDriveFieldCentric);
        //shooter.setDefaultCommand(joystickShooter);
        rangeChecker.setDefaultCommand(detectTag);
    }
}