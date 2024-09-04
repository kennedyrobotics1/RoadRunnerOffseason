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

        drive.setPoseEstimate(new Pose2d(-10, 55, Math.toRadians(90)));

        TrajectorySequence forwardToWoodBack = drive.trajectorySequenceBuilder(new Pose2d(-10, 55, Math.toRadians(0)))
                .lineTo(new Vector2d(48, 55))
                .build();
        Trajectory backToStart = drive.trajectoryBuilder(new Pose2d(48, 55, Math.toRadians(0)))
                .lineTo(new Vector2d(-10, 55))
                .build();

        waitForStart();

        if(isStopRequested()) return;


        drive.turn(Math.toRadians(-90));
        drive.followTrajectorySequence(forwardToWoodBack);
        drive.followTrajectory(backToStart);
    }
}
