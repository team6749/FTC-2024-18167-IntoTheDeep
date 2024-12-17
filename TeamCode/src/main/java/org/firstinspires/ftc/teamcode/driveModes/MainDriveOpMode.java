package org.firstinspires.ftc.teamcode.driveModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;
import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.subsystems.CraneSystem;

public class MainDriveOpMode extends OpMode {
    SampleMecanumDrive drive;
    RobotArm robotArm;
    boolean fastMode = true;
    double SLOW_MODE_STICK_DIVISOR = 3;
    double SUPER_SLOW_MODE_STICK_DIVISOR = 3;
    double DEAD_ZONE = 0.1;
    private boolean backButtonPressedLast = false; // Tracks the previous state of the back button

    private long lastToggleTime = 0;
    private static final long DEBOUNCE_DELAY_MS = 200;
    int loops = 0;
    boolean isBlueAlliance = true;

    public MainDriveOpMode(boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;
    }

    @Override
    public void init() {
        robotArm = new RobotArm(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
    }


    @Override
    public void loop() {
loops++;
        telemetry.addData("loops", loops);

        driveCommands();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        if (gamepad1.a || gamepad1.x) {
            automationCommands();
        } else {
            armCommands();
            telemetry.addData("Lift Angle", robotArm.getCurrentRotationPosition());
            telemetry.addData("liftMotor desired position", robotArm.getRotationDesiredPosition());
            telemetry.addData("liftMotor power", robotArm.getLiftMotorPower());
            telemetry.addData("LiftPIDOutput",robotArm.getPIDOutput());
            telemetry.addData("LiftPowerLimit", robotArm.getLiftPowerLimit());

            telemetry.addData("Extension position", robotArm.getCurrentExtensionPosition());
            telemetry.addData("Extension desired position", robotArm.getCurrentExtensionDesiredPosition());
            telemetry.addData("Extension power", robotArm.getExtensionMotorPower());

            wristCommands();
            telemetry.addData("wrist position", robotArm.getCurrentWristPosition());

            clawCommands();
            telemetry.addData("claw position", robotArm.isClawOpen() ? "OPEN" : "CLOSED");
        }

    }

    private void automationCommands() {
        if (gamepad1.x || gamepad2.x) {
            robotArm.driveMode();
        } else if ((gamepad1.a && gamepad1.right_trigger > 0) ||
                (gamepad2.a && gamepad2.right_trigger > 0)) {
            robotArm.toHighBasket();
        } else if ((gamepad1.a && gamepad1.left_trigger > 0) ||
                (gamepad2.a && gamepad2.left_trigger > 0)) {
            robotArm.toLowBasket();
        }
    }
    private void armCommands() {
            robotArm.publishPID(telemetry);
            if (gamepad1.dpad_up || gamepad2.right_stick_y > DEAD_ZONE) {
                robotArm.raiseArm();
            } else if (gamepad1.dpad_down || gamepad2.right_stick_y < -DEAD_ZONE) {
                robotArm.lowerArm();
            } else {
                robotArm.continueRotating();
            }

            if (gamepad1.right_bumper || gamepad2.left_stick_y > DEAD_ZONE) {
                robotArm.extendArm();
            } else if (gamepad1.left_bumper || gamepad2.left_stick_y < -DEAD_ZONE) {
                robotArm.retractArm();
            } else {
                robotArm.continueExtension();
            }


    }

    private void wristCommands() {
        boolean operationAllowed = true;
        if (gamepad1.right_trigger > 0 || gamepad2.left_stick_x > DEAD_ZONE) {
            operationAllowed = robotArm.rotateWristRight();
        } else if (gamepad1.left_trigger > 0 || gamepad2.left_stick_x < -DEAD_ZONE) {
            operationAllowed = robotArm.rotateWristLeft();
        } else {
           robotArm.rotateWristCenter();
        }
        if (!operationAllowed) {
            gamepad1.rumble(100);

            telemetry.addData("DANGER_ZONE", "WRIST MOVEMENT NOT AVAIL");
        }
    }

    private void clawCommands() {
        //Claw Movement
        if (gamepad1.y || gamepad2.right_bumper) {
            robotArm.openClaw();
        }
        if (gamepad1.b || gamepad2.left_bumper){
            robotArm.closeClaw();
        }
    }

    private void driveCommands() {
        long currentTime = System.currentTimeMillis();
        boolean backButtonPressedNow = gamepad1.back;
        if (backButtonPressedNow && !backButtonPressedLast) {
            if (currentTime - lastToggleTime > DEBOUNCE_DELAY_MS) {
                fastMode = !fastMode;
                lastToggleTime = currentTime;
            }
        }
        // Update the last state to the current state
        backButtonPressedLast = backButtonPressedNow;

        Pose2d weightedStickPose;
        int allianceFlip = isBlueAlliance?1:-1;

        //Make the small movements very slow for most of the stick position
        // Once the stick is moved far, then allow full power
        // Allow a dead zone between switching modes
        int rotationMultiplier = gamepad1.right_stick_x >=0 ? 1 : -1;
        boolean fastTurnMode = gamepad1.right_stick_button;
        double rightStickXAbs = Math.abs(gamepad1.right_stick_x);
        double rotationInput = 0;
        if (rightStickXAbs >= DriveConstants.ROTATION_DEAD_ZONE) {
            if (!fastTurnMode) { //Slow turn mode -- normal
                rotationInput = gamepad1.right_stick_x * (DriveConstants.ROTATION_SLOW_LIMITER - DriveConstants.ROTATION_FEED_FORWARD) + (rotationMultiplier * DriveConstants.ROTATION_FEED_FORWARD);
            } else {
                rotationInput = gamepad1.right_stick_x * DriveConstants.ROTATION_FAST_LIMITER;
            }
        }

        boolean superSlowMode = robotArm.isArmExtendedDangerMode();
        double yStick = gamepad1.left_stick_y;
        double xStick = gamepad1.left_stick_x;
        if (superSlowMode) {
            yStick = gamepad1.left_stick_y / SUPER_SLOW_MODE_STICK_DIVISOR;
            xStick = gamepad1.left_stick_x / SUPER_SLOW_MODE_STICK_DIVISOR;
        } else if (!fastMode) {
            yStick = gamepad1.left_stick_y / SLOW_MODE_STICK_DIVISOR;
            xStick = gamepad1.left_stick_x / SLOW_MODE_STICK_DIVISOR;
        }
        Vector2d input = new Vector2d(
                -yStick,
                -xStick
        ).rotated(-drive.getPoseEstimate().getHeading());
        weightedStickPose = new Pose2d(input.getX(), input.getY(), -rotationInput);

        telemetry.addData("fastSlow", fastMode ? "fast" : "SLOW");
        telemetry.addData("armExtendedSlow", superSlowMode ? "normal" : "SUPER SLOW");
        telemetry.addData("stickX", gamepad1.left_stick_x);
        telemetry.addData("stickY", gamepad1.left_stick_y);
        telemetry.addData("weightedX", weightedStickPose.getX());
        telemetry.addData("weightedY", weightedStickPose.getY());
        telemetry.addData("weightedO", rotationInput);
        telemetry.addData("turnStick", gamepad1.right_stick_x);

        drive.setWeightedDrivePower(weightedStickPose);
        drive.update();
    }
}