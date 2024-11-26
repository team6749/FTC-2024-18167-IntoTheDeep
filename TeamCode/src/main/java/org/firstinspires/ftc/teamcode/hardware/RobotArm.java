package org.firstinspires.ftc.teamcode.hardware;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotArm {
    Motor extenderMotor;
    DcMotorEx liftMotor;

    static final double COUNTS_PER_REVOLUTION = 384.5;
    static final int LIFT_COUNTS_PER_REVOLUTION = 28;// # Adjust this for the specific motor encoder counts per revolution
    static final int GEAR_RATIO = 80;
    static final int CHAIN_RATIO = 40 / 20;
    static final double HIGH_BASKET_ANGLE_RATIO = 0.175;
    public static int EXTENSION_MIN = 0; //TODO - put in a real value
    public static int EXTENSION_DRIVE = (int) (1 * COUNTS_PER_REVOLUTION);
    public static int EXTENSION_MAX = (int) (8 * COUNTS_PER_REVOLUTION); //TODO - put in a real value
    public static int EXTENSION_LOW_BASKET = 20; //TODO - put in a real value
    public static int EXTENSION_HIGH_BASKET = (int) (7.5 * COUNTS_PER_REVOLUTION); //43 in out //TODO - put in a real value
    public static int POSITION_TOLERANCE_EXTENDER = 5;
    public static int EXTEND_INTERVAL = 100;
    public static int WRIST_DANGER_ZONE = 600;
    public static int ROTATE_MIN = 0;
    public static int ROTATE_DRIVE = 10;
    public static int ROTATE_LOW_BASKET = 50; //TODO - put in a real value
    public static int ROTATE_HIGH_BASKET = (int) (LIFT_COUNTS_PER_REVOLUTION * 29 * 0.8);//63 degrees with gear ratio * chain ratio = 160 is about 28 rotations. //TODO - put in a real value
    public static int ROTATE_MAX = (int) (LIFT_COUNTS_PER_REVOLUTION * 29 * 0.8);//63 degrees with gear ratio * chain ratio = 160 is about 28 rotations. //TODO - put in a real value
    public static int ROTATE_INTERVAL = 50;

    public static int POSITION_TOLERANCE_LIFT = 25;
    public static int BASE_ANGLE = 5;

    private int currentRotationDesiredPosition = ROTATE_MIN;
    private int currentExtensionDesiredPosition = EXTENSION_MIN;


    RobotClaw robotClaw;

    public RobotArm(HardwareMap hwMap) {
        extenderMotor = new Motor(hwMap, "extender_motor", Motor.GoBILDA.RPM_435);
        extenderMotor.setRunMode(Motor.RunMode.PositionControl);
        extenderMotor.stopAndResetEncoder();
        extenderMotor.setInverted(true);
        //verticalMotor.setTargetPosition(0);
        extenderMotor.setPositionTolerance(POSITION_TOLERANCE_EXTENDER);
        extenderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extenderMotor.set(0);


        liftMotor = hwMap.get(DcMotorEx.class, "lift_motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPositionTolerance(POSITION_TOLERANCE_LIFT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0);

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

    private void doRotation() {
        if (isAtRotation(currentRotationDesiredPosition)) {
            liftMotor.setPower(0);
        } else {
            if (liftMotor.getCurrentPosition() <= currentRotationDesiredPosition) {
                liftMotor.setPower(0.35);
            } else {
                liftMotor.setPower(-0.05);
            }
        }
    }

    public boolean isAtRotation(int desiredPosition) {
        return Math.abs(liftMotor.getCurrentPosition() - desiredPosition) < POSITION_TOLERANCE_LIFT;
    }


    public void slideToPosition(int desiredPosition) {
        if (EXTENSION_MIN <= desiredPosition &&
                desiredPosition <= EXTENSION_MAX) {
            extenderMotor.setTargetPosition(desiredPosition);
        }
        doExtension();

    }

    private void doExtension() {
        if (extenderMotor.atTargetPosition()) {
            extenderMotor.set(0);
        } else {
            extenderMotor.setPositionCoefficient(0.005);
            extenderMotor.set(.8);
        }
    }

    private boolean isArmLow() {
        return liftMotor.getCurrentPosition() < (ROTATE_DRIVE - POSITION_TOLERANCE_LIFT);
    }

    private boolean isAtDriveRotation() {
        return isAtRotation(ROTATE_DRIVE);
    }

    private boolean isAtDriveExtension() {
        return isExtenderAtPosition(EXTENSION_DRIVE);
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
        slideToPosition(EXTENSION_HIGH_BASKET);
        rotateToHighBasket();
    }

    public void toLowBasket() {
        slideToPosition(EXTENSION_LOW_BASKET);
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

    public boolean isInDangerZone() {
        return extenderMotor.getCurrentPosition() < WRIST_DANGER_ZONE;
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
}
