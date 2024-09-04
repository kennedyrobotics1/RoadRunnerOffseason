package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (group = "drive")

public class obstacleCoursePathFour extends LinearOpMode {

    @Override

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-9, 59, Math.toRadians(0)));

        TrajectorySequence driveUnderObstacle = drive.trajectorySequenceBuilder(new Pose2d(-9, 59, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-5, 30, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-5, -30, Math.toRadians(270)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(driveUnderObstacle);
    }
}