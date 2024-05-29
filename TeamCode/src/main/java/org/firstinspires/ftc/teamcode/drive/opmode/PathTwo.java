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
        // Austin's at 55, 55
        Pose2d startPose = new Pose2d(50, 50, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(0, 50))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(0, 3, Math.toRadians(270)))
                .build();

        // originally -50, may change to -53
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-50, 3, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }
}