package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class PracticePathTestRoadRunner extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-50, 3, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d(-40, 15, Math.toRadians(0)))
                .splineTo(new Vector2d(-5,30), Math.toRadians(90))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(-5,35, Math.toRadians(0)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(33, 45), Math.toRadians(270))
                .build();


                /*.lineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)))
                .lineTo(new Vector2d(35,35))
                .splineTo(new Vector2d(-35,35), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, -35), Math.toRadians(270))
                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(450)))
                .build();
                .splineTo(new Vector2d(35, -30), Math.toRadians(90))
                .splineTo(new Vector2d(45, 35), Math.toRadians(180))
                .splineTo(new Vector2d(-40, 40), Math.toRadians(270))
                .splineTo(new Vector2d(-40, -40), 0)
                .splineTo(new Vector2d(0,0), 0)
        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(45, 35), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-40, 40), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-40, -40), 0)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(0,0), 0)
                .build();
                */
        waitForStart();

        if(isStopRequested()) return;
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        /*
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

         */
    }
}

