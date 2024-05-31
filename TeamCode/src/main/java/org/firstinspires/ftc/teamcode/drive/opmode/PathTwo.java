package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class PathTwo extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // starts at top left corner
        drive.setPoseEstimate(new Pose2d(55, 65, Math.toRadians(180)));

        Trajectory topLeftToMiddleLeft = drive.trajectoryBuilder(new Pose2d(55, 65, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(0, 50))
                .build();

        Trajectory middleLeftToCenter = drive.trajectoryBuilder(topLeftToMiddleLeft.end())
                .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(270)))
                .build();

        Trajectory centerToMiddleBottom = drive.trajectoryBuilder(middleLeftToCenter.end())
                .lineToLinearHeading(new Pose2d(-50, 3, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(topLeftToMiddleLeft);
        drive.followTrajectory(middleLeftToCenter);
        drive.followTrajectory(centerToMiddleBottom);
    }
}