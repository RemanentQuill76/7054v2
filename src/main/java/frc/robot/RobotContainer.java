// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  
  //public RobotContainer() {
    //NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    //NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    //NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Configure the trigger bindings
    //configureBindings();

    //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    //SmartDashboard.putData("Auto Mode", autoChooser);

  //}



  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  //private final SendableChooser<Command> autoChooser;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    //SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      //new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      //new PathConstraints(
        //4.0, 4.0, 
        //Units.degreesToRadians(360), Units.degreesToRadians(540)
      //), 
      //0, 
      //2.0
    //));
    //SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //  new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //  new PathConstraints(
    //    4.0, 4.0, 
    //    Units.degreesToRadians(360), Units.degreesToRadians(540)
    //  ), 
    //  0, 
    //  0
    //));


      //NOTE, BELOW IS AXED DUE TO A UNKNOWN OF the swerve.getPose();


        // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m forward of its current position
    //SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //  //Pose2d currentPose = swerve.getPose();
    //  
    //  // The rotation component in these poses represents the direction of travel
    //  Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //  Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
    //
    //  List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    //  PathPlannerPath path = new PathPlannerPath(
    //    bezierPoints, 
    //    new PathConstraints(
    //      4.0, 4.0, 
    //      Units.degreesToRadians(360), Units.degreesToRadians(540)
    //    ),  
    //    new GoalEndState(0.0, currentPose.getRotation())
    //  );
    //
    //  AutoBuilder.followPathWithEvents(path).schedule();
    //}));

  }

  //public Command getAutonomousCommand() {
  //  return autoChooser.getSelected();
  //}
}
