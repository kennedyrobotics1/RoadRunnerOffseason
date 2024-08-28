package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class izzyPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(new Pose2d(18, -30, Math.toRadians(270)));

        Trajectory forwardToWall= drive.trajectoryBuilder(new Pose2d(18, -30, Math.toRadians(270)))
                .lineTo(new Vector2d(18, -60))
                .build();
        Trajectory forwardToMosaic= drive.trajectoryBuilder(new Pose2d(18, -60, Math.toRadians(0)))
                .lineTo(new Vector2d(50, -60))
                .build();
        Trajectory yeetTheRobotToPark= drive.trajectoryBuilder(new Pose2d(50, -60, Math.toRadians(0)))
                .lineTo(new Vector2d(50, -35))
                .build();

        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(forwardToWall);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forwardToMosaic);
        drive.followTrajectory(yeetTheRobotToPark);
    }
}
