package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Controllers */
        private final Joystick driver = new Joystick(0);
        private final Joystick operator = new Joystick(1);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;
        private final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
        private final int outtakeAxis = XboxController.Axis.kRightTrigger.value;
        private final int armAxis = XboxController.Axis.kRightY.value;

        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        private final JoystickButton slow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
        private final JoystickButton modAbsoluteOffSet = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);
        private final JoystickButton noPID = new JoystickButton(driver, XboxController.Button.kStart.value);

        public final POVButton driverUP;
        public final POVButton driverDOWN;
        public final POVButton driverLEFT;
        public final POVButton driverRIGHT;
        private POVButton operatorUP;
        private POVButton operatorDOWN;
        private POVButton operatorLEFT;
        private POVButton operatorRIGHT;


        /* Subsystems */
        private final Swerve s_Swerve = new Swerve();

        private final Intake intake = new Intake();
        private final Arm arm = new Arm();
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                s_Swerve.setDefaultCommand(
                                new Drive(
                                                s_Swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> getDesiredHeading(),
                                                () -> speedReduction(),
                                                () -> robotCentric.getAsBoolean()));

                intake.setDefaultCommand(
                                new Intakecommand(intake, () -> operator.getRawAxis(intakeAxis),
                                                () -> operator.getRawAxis(outtakeAxis)));
                arm.setDefaultCommand(new ArmCommand(arm, () -> operator.getRawAxis(armAxis)));

                // slow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
                driverUP = new POVButton(driver, 0);
                driverRIGHT = new POVButton(driver, 90);
                driverDOWN = new POVButton(driver, 180);
                driverLEFT = new POVButton(driver, 270);
                operatorUP = new POVButton(operator, 0);
                operatorRIGHT = new POVButton(operator, 90);
                operatorDOWN = new POVButton(operator, 180);
                operatorLEFT = new POVButton(operator, 270);
                // Configure the button bindings
                configureButtonBindings();

                autoChooser.setDefaultOption("Do nothing", new InstantCommand());
                updateAutoChoices();
                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.putData(s_Swerve);
                SmartDashboard.putData(arm);
                SmartDashboard.putData(intake);
                SmartDashboard.putNumber("Pitch", s_Swerve.getPitch());
                Shuffleboard.getTab("Autonomous").add(autoChooser).withSize(2, 1);

        }

        /**
         * Return how much the speed should be reduced.
         * 
         * @return speed reduced percent
         */
        public double speedReduction() {
                if (slow.getAsBoolean()) {
                        SmartDashboard.putBoolean("SpeedReduced", true);
                        return 0.4;
                } else {
                        SmartDashboard.putBoolean("SpeedReduced", false);
                        return 1.0;
                }
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
                noPID.onTrue(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> speedReduction(),
                () -> robotCentric.getAsBoolean()));

                // driverUP.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 0.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // driverDOWN.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 180.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // driverRIGHT.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // -90.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // driverLEFT.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 90.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // operatorUP.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 0.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // operatorDOWN.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 180.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // operatorRIGHT.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // -90.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));
                // operatorLEFT.onTrue(new TurnToAngleCmd(
                // s_Swerve,
                // 90.0,
                // () -> driver.getRawAxis(translationAxis),
                // () -> driver.getRawAxis(strafeAxis),
                // () -> speedReduction()).withTimeout(1.5));

                modAbsoluteOffSet.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        }

        public double getDesiredHeading() {
                if (driverUP.getAsBoolean() || operatorUP.getAsBoolean()) {
                        return 0.0;
                } else if (driverRIGHT.getAsBoolean() || operatorRIGHT.getAsBoolean()) {
                        return -90.0;
                } else if (driverDOWN.getAsBoolean() || operatorDOWN.getAsBoolean()) {
                        return 180.0;
                } else if (driverLEFT.getAsBoolean() || operatorLEFT.getAsBoolean()) {
                        return 90.0;
                } else
                        return 999;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Command Selected from Shuffleboard
                return autoChooser.getSelected();
                // return new Drive1m(s_Swerve);
        }

        public void teleopInit() {
           //     s_Swerve.limitCanCoderTraffic(true);
        }

        public void autonomousInit(){
             //   s_Swerve.limitCanCoderTraffic(true);
        }

        public void disabledInit() {
              //  s_Swerve.limitCanCoderTraffic(false);
        }
        public void updateAutoChoices() {
                /**
                 * Run the Center Start, balance Auto
                 */
                autoChooser.addOption("Center start", (new MoveArmDown(arm).withTimeout(0.7))
                                .andThen(new IntakeOut(intake).withTimeout(2))
                                .andThen(new MoveArmUp(arm).withTimeout(.3))
                                .andThen(new DriveSegment(s_Swerve,
                                                List.of(
                                                                new Translation2d(0, 0),
                                                                new Translation2d(2.7, 0)),
                                                Rotation2d.fromDegrees(180),
                                                Rotation2d.fromDegrees(180),
                                                true,
                                                true).withTimeout(2.5))
                                .andThen(new AutoBalance(s_Swerve)));

                /**
                 * Score And Do Nothing
                 */
                autoChooser.addOption("Score and then Do nothing", new MoveArmDown(arm).withTimeout(0.7)
                                .andThen(new IntakeOut(intake).withTimeout(2)));

                /**
                 * Short Side Start
                 */
                autoChooser.addOption("Short Side Start", new MoveArmDown(arm).withTimeout(0.7)
                                .andThen(new IntakeOut(intake).withTimeout(2))
                                .andThen(new DriveSegment(s_Swerve,
                                                List.of(
                                                                new Translation2d(0, 0),
                                                                new Translation2d(3.1, 0)),
                                                Rotation2d.fromDegrees(180),
                                                Rotation2d.fromDegrees(180),
                                                true,
                                                true)));

                /**
                 * Long Side Start
                 */
                autoChooser.addOption("Long Side Start", (new MoveArmDown(arm).withTimeout(0.7))
                                .andThen(new IntakeOut(intake).withTimeout(2))
                                .andThen(new DriveSegment(s_Swerve,
                                                List.of(
                                                                new Translation2d(0, 0),
                                                                new Translation2d(4.65, 0)),
                                                Rotation2d.fromDegrees(180),
                                                Rotation2d.fromDegrees(180),
                                                true,
                                                true)));

                /**
                 * Long Side Start Pick up 2nd Cube
                 */
                // autoChooser.addOption("Long Side Start Double", new MoveArmDown(arm).withTimeout(0.7)
                //                 .andThen(new IntakeOut(intake)).withTimeout(2)
                //                 .andThen(new DriveSegment(s_Swerve,
                //                                 List.of(
                //                                                 new Translation2d(0, 0),
                //                                                 new Translation2d(4.65, 0)),
                //                                 Rotation2d.fromDegrees(180),
                //                                 Rotation2d.fromDegrees(180),
                //                                 false,
                //                                 true))
                //                 .andThen(new IntakeOut(intake).withTimeout(2))
                //                 .andThen(new DriveSegment(s_Swerve,
                //                                 List.of(
                //                                                 new Translation2d(4.65, 0),
                //                                                 new Translation2d(5.68, 0)),
                //                                 Rotation2d.fromDegrees(180),
                //                                 Rotation2d.fromDegrees(0),
                //                                 true,
                //                                 false)));

                // Second level scoring - LongSideStart

                autoChooser.addOption("Long Side Start Second Level", (new MoveArmUp(arm).withTimeout(0.5))
                                .andThen(new IntakeOut(intake).withTimeout(4))
                                .andThen(new MoveArmDown(arm).withTimeout(.75))
                                .andThen(new DriveSegment(s_Swerve,
                                                List.of(
                                                                new Translation2d(0, 0),
                                                                new Translation2d(4.8, 0)),
                                                              
                                                Rotation2d.fromDegrees(0),
                                                Rotation2d.fromDegrees(0),
                                                true,
                                                false))
                                .andThen(new IntakeIn(intake).withTimeout(4)));
                       
        }
}
