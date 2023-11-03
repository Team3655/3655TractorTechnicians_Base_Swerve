// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	private static final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

	private final SendableChooser<Command> autoChooser;

	private final CommandJoystick driveJoystick = new CommandJoystick(
			OperatorConstants.kDriveJoystickPort);
	private final CommandJoystick turnJoystick = new CommandJoystick(
			OperatorConstants.kTurnJoystickPort);
	private final CommandGenericHID operatorController = new CommandGenericHID(
			OperatorConstants.kOperatorControllerPort);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// region Def Auto
		PPHandler.configAutoBuilder();

		autoChooser = AutoBuilder.buildAutoChooser(null);
		Shuffleboard.getTab("Auto").add(autoChooser);
		// endregion

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		// region Drive Commands
		driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

		programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

		// Swerve Drive command is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new TeleopDriveCommand(
						() -> -driveJoystick.getY() - programmerController.getLeftY(),
						() -> -driveJoystick.getX() - programmerController.getLeftX(),
						() -> -turnJoystick.getX() - programmerController.getRightX()));
		// endregion
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		driveSubsystem.setHeading(180);
		Timer.delay(0.05);
		// the command to be run in autonomous
		return autoChooser.getSelected();
	}
}