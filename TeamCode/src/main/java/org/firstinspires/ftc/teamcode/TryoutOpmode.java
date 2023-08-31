package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TryoutOpmode: TeleOp", group="TeleOp")
public class TryoutOpmode extends LinearOpMode {

    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean timerStarted = false;
    private double timerStartTime = 0;
    private boolean fieldOriented = true;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = gamepad1.left_stick_x;
            double turnInput = gamepad1.right_stick_x;

            // Power scaling modes - Left Bumper (slow mode), Right bumper (Super Slow mode)
            if (gamepad1.left_bumper) {
                driveInput *= 0.5;
                strafeInput *= 0.5;
                turnInput *= 0.5;
            } else if (gamepad1.right_bumper) {
                driveInput *= 0.25;
                strafeInput *= 0.25;
                turnInput *= 0.25;
            }

            // Quick Turn - Turns twice faster, Press Right Trigger

            if (gamepad1.right_trigger > 0.5) {
                turnInput *= 2.0;
            }

            // Strafe Mode - Makes easier to strafe, Press button B
            if (gamepad1.b) {
                double robotAngle = drive.getPoseEstimate().getHeading();
                strafeInput = 0.5 * Math.sin(robotAngle);  // The value "0.5" defines the speed and can be adjusted.
                driveInput = 0.5 * Math.cos(robotAngle);
            }

            // Point Turn Mode - Point turns, Press button X
            if (gamepad1.x) {
                double desiredHeading = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                double currentHeading = drive.getPoseEstimate().getHeading();
                turnInput = desiredHeading - currentHeading;
            }

            // Toggle field-oriented drive - Press Button A
            if (gamepad1.a) {
                fieldOriented = !fieldOriented;
                while (gamepad1.a) {} // Debouncing
            }

            // Brake Mode - Instantly brakes robot, Press Button Y
            if (gamepad1.y) {
                driveInput = 0;
                strafeInput = 0;
                turnInput = 0;
            }

            // Reset Odometry - Reset the Odometry of Robot, Press Dpad Up
            if (gamepad1.dpad_up) {
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
            }

            // Field-Oriented control adjustments
            if (fieldOriented) {
                double robotAngle = drive.getPoseEstimate().getHeading();
                double temp = driveInput * Math.cos(robotAngle) - strafeInput * Math.sin(robotAngle);
                strafeInput = driveInput * Math.sin(robotAngle) + strafeInput * Math.cos(robotAngle);
                driveInput = temp;
            }

            Pose2d poseVelo = new Pose2d(driveInput, strafeInput, turnInput);
            drive.setWeightedDrivePower(poseVelo);

            // Start/Stop Timer with the 'dpad_down' button
            if (gamepad1.dpad_down && !timerStarted) {
                timerStartTime = getRuntime();
                timerStarted = true;
            } else if (gamepad1.dpad_down && timerStarted) {
                timerStarted = false;
            }

            // Telemetry updates
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("globalX", poseEstimate.getX());
            telemetry.addData("globalY", poseEstimate.getY());
            telemetry.addData("globalHeading", poseEstimate.getHeading());
            telemetry.addData("OpMode Runtime", getRuntime());
            if (timerStarted) {
                telemetry.addData("Timer", getRuntime() - timerStartTime);
            }
            telemetry.update();
        }
    }

    // Method to get runtime
    public double getRuntime() {
        return runtime.seconds();
    }
}
