package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterAuton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

    // JOYSTICKS, SUBSYSTEMS, AND COMMANDS MUST ALL BE PRIVATE AND FINAL

    /*
     * Joystick
     * - The objects in this catagory are joysticks, they should be either a
     * CommandXboxController, XboxController, or a Joystick
     * - The joystick objects handle handle all input from joysticks which are
     * plugged in to the Driver Station
     * - The port of the joystick is selected in the Driver Station, generally we
     * have the drive controller in port 0 while the operator controller uses port 1
     */
    private final CommandXboxController driver = new CommandXboxController(
            Constants.ControllerConstants.kDriveControllerPort);
    private final CommandXboxController operator = new CommandXboxController(
            Constants.ControllerConstants.kOperatorControllerPort);

    /*
     * Subsystems
     * - This catagory is for all subsystems which the robot has.
     * - Subsystems should have methods which do low-level things such as running a
     * motor, there shouldn't be complicated algorithms
     */
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    /*
     * Commands
     * - This catagory is for commands, the commands can either be lambda instant or
     * run commands, or they can be classes
     */
    // private final SwerveDriveFieldCentric swerveDriveFieldCentric = new
    // SwerveDriveFieldCentric(driver, driveBase);
    // private final Command joystickShooter = new InstantCommand(() ->
    // shooter.run(operator.getLeftY(), operator.getLeftY()), shooter);
    // private final Command runShooter = new RunCommand(() -> shooter.run(11.5,
    // 10.5), shooter); // change values later
    private final Command runIntake = new RunCommand(() -> {
        intake.run(operator.getLeftY());
    }, intake);

    private final IntakeNote intakeNote = new IntakeNote(intake);
    // private final Command runIntakeAuton = new RunCommand(() -> intake.run(0.0),
    // intake);
    // private final SendableChooser<Command> autonomousChooser = new
    // SendableChooser<>();

    /*
     * Drivebase
     * - This category is for TunerX Command Based Drivetrain initialization.
     */
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add
                                                                                                                       // a
                                                                                                                       // 10%
                                                                                                                       // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle fieldAngle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

    /*
     * Autos
     */

    private final PathPlannerAuto oneNoteLeftAuto = new PathPlannerAuto("OneNoteLeftAuto");

    /**
     * Creates new RobotContainer and configures auton and buttons
     */
    public RobotContainer() {
        NamedCommands.registerCommand("RunIntake", runIntake);
        configureButtonBindings();
        configureAutoSelector();
        registerCommands();
    }

    /**
     * Registers commands in the path planner system, these can be used when running
     * paths
     */
    private void registerCommands() {
        NamedCommands.registerCommand("RunShooterAuton", new RunShooterAuton(shooter, intake));
    }

    /**
     * This method is for configuring the button bindings on the joysticks
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * SwerveConstants.MaxSpeed) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                        // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X
                                                                                      // (left)
                        .withRotationalRate(-driver.getRightX() * SwerveConstants.MaxAngularRate) // Drive
                                                                                                  // counterclockwise
                                                                                                  // with negative X
                                                                                                  // (left)
                ));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        /* Bindings for drivetrain characterization */
        /*
         * These bindings require multiple buttons pushed to swap between quastatic and
         * dynamic
         */
        /*
         * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
         */
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driver.x().whileTrue(
                drivetrain.applyRequest(
                        () -> fieldAngle.withVelocityX(-driver.getLeftY() * SwerveConstants.MaxSpeed)
                                .withVelocityY(-driver.getLeftX() * SwerveConstants.MaxSpeed)
                                .withTargetDirection(new Rotation2d(0))
                                .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                                .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)));

        /* Bindings for operator */
        operator.back().and(operator.y()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
        operator.back().and(operator.x()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        operator.start().and(operator.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        operator.start().and(operator.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        operator.rightBumper()
                .whileTrue(new RunShooter(shooter, ShooterConstants.topShooterRPM, ShooterConstants.bottomShooterRPM));
        operator.leftBumper().onTrue(intakeNote);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * This method is for configuring the auton chooser
     */
    private void configureAutoSelector() {
        // SmartDashboard.putData("Auton", autonomousChooser);
    }

    /**
     * This method gets callled in Robot to schedule the auton command, so whatever
     * commond this method returns will be the one running during the autonomous
     * period
     * 
     * @return Auton command
     */
    public Command getAutonomousCommand() {
        return oneNoteLeftAuto;
        // return new TranslateDistance(driveBase, 1, 0);
    }

    /**
     * Sets default commands to be used for teleop
     */
    public void setDefaultCommands() {
        // shooter.setDefaultCommand(new RunShooter(shooter,1600, 1600));
        intake.setDefaultCommand(runIntake);
    }
}