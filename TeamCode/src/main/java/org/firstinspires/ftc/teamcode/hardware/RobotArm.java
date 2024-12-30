package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class RobotArm {
    Motor extenderMotor;
    DcMotorEx liftMotor;

    public static double EXTENDER_KS = 0.05;
    public static double EXTENDER_KV = 0.5;
    public static double EXTENDER_KA = 0.5;
    public static double EXTENDER_POS_CF_CLOSE = 0.007;
    public static double EXTENDER_POS_CF_FAR = 0.014;
    static double COUNTS_PER_REVOLUTION = 384.5;
    static final int LIFT_COUNTS_PER_REVOLUTION = 28;// # Adjust this for the specific motor encoder counts per revolution
    static final int GEAR_RATIO = 80;
    static final int CHAIN_RATIO = 40 / 15; //small gear is 15, large gear is 40
    static final double HIGH_BASKET_ANGLE_RATIO = 0.175;
    public static int EXTENSION_MIN = 10; //TODO - put in a real value
    public static int EXTENSION_DRIVE = (int) (1 * COUNTS_PER_REVOLUTION);
    public static int EXTENSION_MAX = (int) (8.5 * COUNTS_PER_REVOLUTION); //TODO - put in a real value
    public static int EXTENSION_LOW_BASKET = (int) (4.125 * COUNTS_PER_REVOLUTION);
    public static int EXTENSION_HIGH_BASKET = (int) (7.5 * COUNTS_PER_REVOLUTION); //43 in out
    public static int POSITION_TOLERANCE_EXTENDER = 10;
    public static int EXTEND_INTERVAL = 100;
    public static double EXTENDER_POWER_MAX_FAR = 1.0;
    public static double EXTENDER_POWER_MAX_CLOSE = 0.5;
    private static long MAX_MILLIS_EXTENDER_TIME = 2000;
    public static int WRIST_DANGER_ZONE = 600;
    public static int ROTATE_MIN = 0;
    public static int ROTATE_DRIVE = 140;
    public static int ROTATE_LOW_BASKET = 800;
    public static int ROTATE_HIGH_BASKET = 1000;//(int) (LIFT_COUNTS_PER_REVOLUTION * 29.5 * 0.8);//63 degrees with gear ratio * chain ratio = 160 is about 28 rotations. //TODO - put in a real value
    //ORIG 28
    public static int ROTATE_MAX = 1000;//(int) (LIFT_COUNTS_PER_REVOLUTION * 29.5 * 0.8);//63 degrees with gear ratio * chain ratio = 160 is about 28 rotations. //TODO - put in a real value
    public static int ROTATE_INTERVAL = 50;
    public static double MAX_DOWN_POWER = 0.28;
    private static double EXTENDED_POWER_LIMIT = 1.0;
    private static double RETRACTED_POWER_LIMIT = 0.35;
    private PIDController liftPID;

    private PIDController liftPID2;
    public static double p = 0.005;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0.000009;


    private long lastTimeExtenderHitPosition = System.currentTimeMillis();

    public static int POSITION_TOLERANCE_LIFT = 28;
    public static int BASE_ANGLE = 5;

    private int currentRotationDesiredPosition = ROTATE_MIN;
    private int currentExtensionDesiredPosition = EXTENSION_MIN;

    public static TouchSensor lowerLimitExtensionSwitch = null;

    RobotClaw robotClaw;

    public RobotArm(HardwareMap hwMap) {
        extenderMotor = new Motor(hwMap, "extender_motor", Motor.GoBILDA.RPM_435);
        extenderMotor.setRunMode(Motor.RunMode.PositionControl);
        extenderMotor.setInverted(true);
        extenderMotor.setPositionTolerance(POSITION_TOLERANCE_EXTENDER);
        extenderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extenderMotor.stopAndResetEncoder();
        extenderMotor.setTargetPosition(0);
//        extenderMotor.setFeedforwardCoefficients(EXTENDER_KS, EXTENDER_KV, EXTENDER_KA);
//        extenderMotor.setPositionCoefficient(EXTENDER_POS_CF);
        extenderMotor.set(0);


        liftMotor = hwMap.get(DcMotorEx.class, "lift_motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPositionTolerance(POSITION_TOLERANCE_LIFT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setPower(0);

        lowerLimitExtensionSwitch = hwMap.get(TouchSensor.class, "lowerLimitExtensionSwitch");

        liftPID = new PIDController(0.15, 0.1, 0.0);
        liftPID.setTolerance(POSITION_TOLERANCE_LIFT);


        liftPID2 = new PIDController(0.15, 0.1, 0.0);
        liftPID2.setTolerance(POSITION_TOLERANCE_LIFT);

        robotClaw = new RobotClaw(hwMap);
    }


    public void rotateToHighBasket() {
        rotateToPosition(ROTATE_HIGH_BASKET);
    }

    public void rotateToLowBasket() {
        rotateToPosition(ROTATE_LOW_BASKET);
    }

    public void rotateToBase() {
        rotateToPosition(ROTATE_MIN);
    }


    public void rotateToPosition(int desiredPosition) {
        if (ROTATE_MIN <= desiredPosition &&
                desiredPosition <= ROTATE_MAX) {
            currentRotationDesiredPosition = desiredPosition;
            liftMotor.setTargetPosition(desiredPosition);
            doRotation();
        } else {
            liftMotor.setPower(0);
        }
    }

//    private void doRotation() {
//        if (isAtRotation(currentRotationDesiredPosition)) {
//            liftMotor.setPower(0);
//        } else {
//            double powerLimit = 0;
//            if (liftMotor.getCurrentPosition() <= currentRotationDesiredPosition) {
//
//                // Get the PID output
//                double pidOutput = liftPID.calculate(liftMotor.getCurrentPosition(), currentRotationDesiredPosition);
//
//                // Determine the power limit based on the extension
//                //double powerLimit = ((double)(getCurrentExtensionPosition()/EXTENSION_MAX) * (EXTENDED_POWER_LIMIT - RETRACTED_POWER_LIMIT)) + RETRACTED_POWER_LIMIT;
//                powerLimit = .7;
////                if (getCurrentExtensionPosition() < EXTENSION_MAX * 0.5) {
////                    powerLimit = ((double)(getCurrentExtensionPosition() / EXTENSION
////                    _MAX) * 2 * (EXTENDED_POWER_LIMIT - RETRACTED_POWER_LIMIT)) + RETRACTED_POWER_LIMIT;
////                } else {
////                    // Increase the power limit gradually as the arm extends further
////                    powerLimit = 1;// Math.min(EXTENDED_POWER_LIMIT, EXTENDED_POWER_LIMIT + (getCurrentExtensionPosition() - EXTENSION_MAX * 0.6) * 4);
////                }
//
//
//                // Scale PID output to the power limit
//                double adjustedPower = Math.max(-powerLimit, Math.min(pidOutput, powerLimit));
//                liftMotor.setPower(adjustedPower);
//            } else {
//                liftMotor.setPower(-MAX_DOWN_POWER);
////                powerLimit = MAX_DOWN_POWER;
//            }
//
//
//        }


//    }

    private void doRotation () {
        // Configure the PID controller
        liftPID2.setPID(p, i, d);
        double angleInRadians = Math.toRadians(((double) liftMotor.getCurrentPosition() / ROTATE_MAX) * .75);
        double armLengthInMeters = .33 + (.94 * getCurrentExtensionPosition() / EXTENSION_MAX);
        double feedForward = f;// * armLengthInMeters * Math.cos(angleInRadians);
        liftPID2.setF(feedForward);
        double powerLimit = .7;
        double pidOutput = liftPID2.calculate(liftMotor.getCurrentPosition(), currentRotationDesiredPosition);
        double adjustedPower = Math.max(-powerLimit, Math.min(pidOutput, powerLimit));
        liftMotor.setPower(adjustedPower);
    }
    public void publishPID(Telemetry telemetry) {

        // Configure the PID controller
        liftPID2.setPID(p, i, d);
        double angleInRadians = Math.toRadians(((double) liftMotor.getCurrentPosition() / ROTATE_MAX) * .75);
        double armLengthInMeters = .33 + (.94 * getCurrentExtensionPosition() / EXTENSION_MAX);
        double feedForward = f;// * armLengthInMeters * Math.cos(angleInRadians);
        liftPID2.setF(feedForward);
        double powerLimit = .7;
        double pidOutput = liftPID2.calculate(liftMotor.getCurrentPosition(), currentRotationDesiredPosition);
        double adjustedPower = Math.max(-powerLimit, Math.min(pidOutput, powerLimit));

        telemetry.addData("PID POWER", adjustedPower);
        telemetry.addData("PID OUTPUT", pidOutput);
        telemetry.addData("FF OUTPUT", feedForward);
        telemetry.addData("PID ANGLE", angleInRadians);
        telemetry.addData("PID ARM LENGTH", armLengthInMeters);
        telemetry.addData("PID FEED FORWARD", feedForward);
    }

    private void doRotation2() {
        if (isAtRotation(currentRotationDesiredPosition)) {
            liftMotor.setPower(0);
        } else {
            // Configure the PID controller
            double angleInRadians = Math.toRadians(((double) liftMotor.getCurrentPosition() / ROTATE_MAX) * .75);
            double armLengthInMeters = .33 + (.94 * getCurrentExtensionPosition() / EXTENSION_MAX);
            double feedForward = 0.2 * armLengthInMeters * Math.cos(angleInRadians);
            liftPID.setF(feedForward);
            double powerLimit = .7;
            double pidOutput = liftPID.calculate(liftMotor.getCurrentPosition(), currentRotationDesiredPosition);
            double adjustedPower = Math.max(-powerLimit, Math.min(pidOutput, powerLimit));
            liftMotor.setPower(adjustedPower);


        }


    }

    public boolean isAtRotation(int desiredPosition) {
        return Math.abs(liftMotor.getCurrentPosition() - desiredPosition) < POSITION_TOLERANCE_LIFT;
    }


    public void slideToPosition(int desiredPosition) {
        if (EXTENSION_MIN <= desiredPosition &&
                desiredPosition <= EXTENSION_MAX) {
            extenderMotor.setTargetPosition(desiredPosition);
            if (desiredPosition != currentExtensionDesiredPosition) {
                //Reset our timer for last time that we were at position;
                lastTimeExtenderHitPosition = System.currentTimeMillis();
            }
            currentExtensionDesiredPosition = desiredPosition;
        }
        doExtension();

    }

    private void doExtension() {
        if (lowerLimitExtensionSwitch.isPressed() && Math.abs(extenderMotor.getCurrentPosition()) > POSITION_TOLERANCE_EXTENDER) {
            extenderMotor.set(0);
            extenderMotor.stopAndResetEncoder();
            extenderMotor.setTargetPosition(0);
            currentExtensionDesiredPosition = 0;
        }
        if (extenderMotor.atTargetPosition() ||
                (lowerLimitExtensionSwitch.isPressed() && currentExtensionDesiredPosition == 0)) {
            lastTimeExtenderHitPosition = System.currentTimeMillis();
            extenderMotor.set(0);
        } else {
            if (Math.abs(currentExtensionDesiredPosition-extenderMotor.getCurrentPosition()) > 300) {
                extenderMotor.setPositionCoefficient(EXTENDER_POS_CF_FAR);
                extenderMotor.set(EXTENDER_POWER_MAX_FAR);
            } else {
                extenderMotor.setPositionCoefficient(EXTENDER_POS_CF_CLOSE);
                extenderMotor.set(EXTENDER_POWER_MAX_CLOSE);
            }
        }
    }

    private boolean isArmLow() {
        return liftMotor.getCurrentPosition() < (ROTATE_DRIVE - POSITION_TOLERANCE_LIFT);
    }

    private boolean isAtDriveRotation() {
        return isAtRotation(ROTATE_DRIVE);
    }

    private boolean isAtDriveExtension() {
        //We give this a little more tolerance -- because we just need to be close.
        return Math.abs(extenderMotor.getCurrentPosition() - EXTENSION_DRIVE) <= (POSITION_TOLERANCE_EXTENDER * 2.5);
    }

    public void driveMode() {
        //If the arm is low, get the arm up first, then retract

        if (isArmLow()) {
            rotateToPosition(ROTATE_DRIVE);
            if (isAtDriveRotation()) {
                slideToPosition(EXTENSION_DRIVE);
            }
        } else {
            slideToPosition(EXTENSION_DRIVE);
            if (isAtDriveExtension()) {
                rotateToPosition(ROTATE_DRIVE);
            }
        }
    }

    public void toHighBasket() {
        rotateToHighBasket();
        if(isAtRotation(ROTATE_HIGH_BASKET)) {
            slideToPosition(EXTENSION_HIGH_BASKET);
        }
    }

    public void toLowBasket() {
        rotateToLowBasket();
        if(isAtRotation(ROTATE_LOW_BASKET)) {
            slideToPosition(EXTENSION_LOW_BASKET);
        }
    }


    public boolean isFullyRetracted() {
        return isExtenderAtPosition(EXTENSION_MIN);
    }

    public boolean isAtLowBasketPosition() {
        return isExtenderAtPosition(EXTENSION_LOW_BASKET);
    }

    public boolean isAtHighBasketPosition() {
        return isExtenderAtPosition(EXTENSION_HIGH_BASKET);
    }

    private boolean isExtenderAtPosition(int positionToCheck) {
        return Math.abs(extenderMotor.getCurrentPosition() - positionToCheck) <= POSITION_TOLERANCE_EXTENDER;
    }

    public void stop() {
        extenderMotor.set(0);
        liftMotor.setPower(0);
    }

    public int getCurrentExtensionPosition() {
        return extenderMotor.getCurrentPosition();
    }

    public int getCurrentRotationPosition() {
        return liftMotor.getCurrentPosition();
    }

    public int getRotationDesiredPosition(){
        return currentRotationDesiredPosition;
    }
    public double getPIDOutput() {
        return liftPID.calculate(liftMotor.getCurrentPosition(),currentRotationDesiredPosition);
    }
    public double getLiftMotorPower(){
        return liftMotor.getPower();
    }
    public  double getLiftPowerLimit() {
        return ((double)(getCurrentExtensionPosition()/EXTENSION_MAX) * (EXTENDED_POWER_LIMIT - RETRACTED_POWER_LIMIT)) + RETRACTED_POWER_LIMIT;
    }
    public boolean isInDangerZone() {
        return extenderMotor.getCurrentPosition() < WRIST_DANGER_ZONE;
    }
    public boolean isArmExtendedDangerMode() {
        return extenderMotor.getCurrentPosition() > WRIST_DANGER_ZONE;
    }

    public boolean isClawOpen() {
        return robotClaw.isOpen();
    }

    public double getCurrentWristPosition() {
        return robotClaw.getWristPosition();
    }

    public boolean rotateWristRight() {
        if (isInDangerZone()) {
            return false;
        } else {
            robotClaw.toWristRight();
        }
        return true;
    }

    public boolean rotateWristLeft() {
        if (isInDangerZone()) {
            return false;
        } else {
            robotClaw.toWristLeft();
        }
        return true;
    }

    public void rotateWristCenter() {
        robotClaw.toWristCenter();
    }

    public void openClaw() {
        robotClaw.openClaw();
    }

    public void closeClaw() {
        robotClaw.closeClaw();
    }

    public void raiseArm() {
        rotateToPosition(Math.min(getCurrentRotationPosition() + ROTATE_INTERVAL, ROTATE_MAX));
    }

    public void lowerArm() {
        rotateToPosition(Math.max(getCurrentRotationPosition() - ROTATE_INTERVAL, ROTATE_MIN));
    }

    public void extendArm() {
        slideToPosition(Math.min(getCurrentExtensionPosition() + EXTEND_INTERVAL, EXTENSION_MAX));
    }

    public void retractArm() {
        slideToPosition(Math.max(getCurrentExtensionPosition() - EXTEND_INTERVAL, EXTENSION_MIN));
    }

    public void continueRotating() {
        doRotation();
    }

    public void continueExtension() {
        doExtension();
    }

    public int getCurrentExtensionDesiredPosition() {
        return currentExtensionDesiredPosition;
    }

    public double getExtensionMotorPower() {
        return extenderMotor.get();
    }


}
