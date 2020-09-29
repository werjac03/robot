package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class SixballTrench {

    public static SwerveDrive swerveDrive = new SwerveDrive();

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(SwerveDriveConstants.kDriveKinematics);


        Trajectory StarttoTrench = TrajectoryGenerator.generateTrajectory(
            // start at the origin facing +x direction
            new Pose2d(0, 0, new Rotation2d((Math.PI) / 2.)),
            List.of(new Translation2d(0,0)
            ),
            new Pose2d(0, 0, new Rotation2d((Math.PI) / 2.)), config);

        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(StarttoTrench, (0),
            swerveDrive::getPose,
            SwerveDriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), theta,

            swerveDrive::setModuleStates,

            swerveDrive

        );

        public auto = new InstantCommand();
        return auto;
    }
}
    