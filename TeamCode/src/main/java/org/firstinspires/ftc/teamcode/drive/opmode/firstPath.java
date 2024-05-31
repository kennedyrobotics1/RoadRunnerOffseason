package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous (group = "drive")
public class firstPath extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starts one tile forward from bottom right corner, angled 90 degrees to the left
        drive.setPoseEstimate(new Pose2d(-30, -50, Math.toRadians(90)));

        Trajectory lineToTopRightCorner = drive.trajectoryBuilder(new Pose2d(-30, -50, Math.toRadians(90)))
                        .lineTo(new Vector2d(59, -47))
                                .build();
        Trajectory lineToTopLeftCornerAndRotation = drive.trajectoryBuilder(lineToTopRightCorner.end())
                        .lineToLinearHeading(new Pose2d(55, 65, Math.toRadians(180)))
                                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(lineToTopRightCorner);
        drive.followTrajectory(lineToTopLeftCornerAndRotation);


    }
}
