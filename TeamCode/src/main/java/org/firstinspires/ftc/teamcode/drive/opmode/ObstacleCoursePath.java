package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (group = "drive")
public class ObstacleCoursePath extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // first part

        drive.setPoseEstimate(new Pose2d(-60, 60, Math.toRadians(270)));
        Trajectory trajectoryGoForward = drive.trajectoryBuilder(new Pose2d(-60, 60, Math.toRadians(270)))
                .splineTo(new Vector2d(-57.3, 29), Math.toRadians(270))
                .build();

        Trajectory trajectoryTurnLeft = drive.trajectoryBuilder(trajectoryFirstPart.end())
                .splineTo(new Vector2d(5, 10), Math.toRadians(0))
                .build();

        // second part

        Trajectory backUpAfterTrucking = drive.trajectoryBuilder(new Pose2d(5, 10, Math.toRadians(0)))
                .lineTo(new Vector2d(-10, 10))
                .build();

        Trajectory forwardToBlueWall = drive.trajectoryBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .lineTo(new Vector2d(-10, 55))
                .build();

        // third part

        TrajectorySequence forwardToWoodBack = drive.trajectorySequenceBuilder(new Pose2d(-10, 55, Math.toRadians(0)))
                .lineTo(new Vector2d(48, 55))
                .build();

        Trajectory backToStart = drive.trajectoryBuilder(new Pose2d(48, 55, Math.toRadians(0)))
                .lineTo(new Vector2d(-10, 55))
                .build();

        // fourth part

        TrajectorySequence driveUnderObstacle = drive.trajectorySequenceBuilder(new Pose2d(-10, 55, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-5, 30, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-5, -30, Math.toRadians(270)))
                .build();

        // fifth part

        Trajectory forwardToWall = drive.trajectoryBuilder(new Pose2d(-5, -30, Math.toRadians(270)))
                .lineTo(new Vector2d(-5, -57))
                .build();

        Trajectory forwardToMosaic = drive.trajectoryBuilder(new Pose2d(-5, -57, Math.toRadians(90)))
                .lineTo(new Vector2d(40, -57))
                .build();

        Trajectory yeetTheRobotToPark = drive.trajectoryBuilder(new Pose2d(40, -57, Math.toRadians(90)))
                .lineTo(new Vector2d(40, -35))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        // first chunk

        drive.followTrajectory(trajectoryGoForward);
        drive.followTrajectory(trajectoryTurnLeft);

        // second part

        drive.followTrajectory(backUpAfterTrucking);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forwardToBlueWall);

        // third part

        drive.turn(Math.toRadians(-90));
        drive.followTrajectorySequence(forwardToWoodBack);
        drive.followTrajectory(backToStart);

        // fourth part

        drive.followTrajectorySequence(driveUnderObstacle);

        // fifth part
        
        drive.followTrajectory(forwardToWall);
        drive.turn(Math.toRadians(180) + 1e-6);
        drive.followTrajectory(forwardToMosaic);
        drive.followTrajectory(yeetTheRobotToPark);
    }
}