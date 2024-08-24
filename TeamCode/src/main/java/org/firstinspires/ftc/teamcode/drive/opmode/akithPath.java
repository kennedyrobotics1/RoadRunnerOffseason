package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class akithPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory fowardToWoodBack = drive.trajectoryBuilder(new Pose2d(-9, 50, Math.toRadians(0)))
                        .lineTo(new Vector2d(48, 50))
                        .lineTo(new Vector2d(-9, 50))
                                .build();



        waitForStart();

        if(isStopRequested()) return;


        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(fowardToWoodBack);

    }

}