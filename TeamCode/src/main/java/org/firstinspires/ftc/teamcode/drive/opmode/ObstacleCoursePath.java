package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")

public class ObstacleCoursePath extends LinearOpMode {


    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-60, 60, Math.toRadians(270)));

        Trajectory trajectoryFirstPart = drive.trajectoryBuilder(new Pose2d(-60, 60, Math.toRadians(270)))
                .splineTo(new Vector2d(-57.3, 29), Math.toRadians(270))
                .build();

        Trajectory trajectorySecondPart = drive.trajectoryBuilder(trajectoryFirstPart.end())
                .splineTo(new Vector2d(7.5, 12), Math.toRadians(0))
                .build();







        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectoryFirstPart);
        drive.followTrajectory(trajectorySecondPart);

    }
}


