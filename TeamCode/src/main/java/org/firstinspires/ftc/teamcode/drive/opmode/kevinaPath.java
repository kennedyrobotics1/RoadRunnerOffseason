package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class kevinaPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // delete this
        drive.setPoseEstimate(new Pose2d(5, 10, Math.toRadians(0)));

        Trajectory backUpAfterTrucking = drive.trajectoryBuilder(new Pose2d(5, 10, Math.toRadians(0)))
                .lineTo(new Vector2d(-10, 10))
                .build();

        Trajectory forwardToBlueWall = drive.trajectoryBuilder(new Pose2d(-10, 10, Math.toRadians(90)))
                .lineTo(new Vector2d(-10, 55))
                .build();

        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(backUpAfterTrucking);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forwardToBlueWall);
    }
}
