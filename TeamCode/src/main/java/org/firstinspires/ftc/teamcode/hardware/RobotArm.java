package org.firstinspires.ftc.teamcode.hardware;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotArm {
    Motor extenderMotor;
    DcMotorEx liftMotor;

    static final double COUNTS_PER_REVOLUTION = 384.5 ;// # Adjust this for the specific motor encoder counts per revolution
    public static int EXTENSION_BASE = 5; //TODO - put in a real value

    public static int EXTENSION_LOW_BASKET = 20; //TODO - put in a real value
    public static int EXTENSION_HIGH_BASKET = (int) (1 * COUNTS_PER_REVOLUTION); //43 in out //TODO - put in a real value
    public static int POSITION_TOLERANCE_EXTENDER = 5;

    public static int ROTATE_BASE = 0;
    public static int ROTATE_LOW_BASKET = 50; //TODO - put in a real value
    //public static int ROTATE_HIGH_BASKET = 61.16 degrees; //TODO - put in a real value
    public static int POSITION_TOLERANCE_LIFT = 5;
    public static int BASE_ANGLE = 5;
    public RobotArm(HardwareMap hwMap) {
        extenderMotor = new Motor(hwMap, "extender_motor", Motor.GoBILDA.RPM_435);
        extenderMotor.setRunMode(Motor.RunMode.PositionControl);
        extenderMotor.resetEncoder();
        extenderMotor.setInverted(true);
        //verticalMotor.setTargetPosition(0);
        extenderMotor.setPositionTolerance(POSITION_TOLERANCE_EXTENDER);
        extenderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extenderMotor.set(0);

        // Set the motor to run using encoders
//        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor = hwMap.get(DcMotorEx.class, "lift_motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPositionTolerance(POSITION_TOLERANCE_LIFT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0);

    }





    /*public void rotateToHighBasket() {
        rotateToPosition(ROTATE_HIGH_BASKET);
    *}

    public void rotateToLowBasket() {
        rotateToPosition(ROTATE_LOW_BASKET);
    }

    public void rotateToBase() {
        rotateToPosition(ROTATE_BASE);
    }
*


    public void rotateToPosition(int desiredPosition){
        liftMotor.setTargetPosition(desiredPosition);
        if (liftMotorAtSetPoint(desiredPosition)) {
            liftMotor.setPower(0);
        }
        else{
            liftMotor.setPower(1);
        }
    }
*/
    /*public boolean liftMotorAtSetPoint(int desiredPosition) {
        return Math.abs(liftMotor.getCurrentPosition()-desiredPosition) < POSITION_TOLERANCE_LIFT;
    }
*/
    public void slideToPosition(int desiredPosition, float triggerPressure) {

            extenderMotor.setTargetPosition(desiredPosition);
            if (extenderMotor.atTargetPosition()) {
                extenderMotor.set(0);
            } else {
                extenderMotor.setPositionCoefficient(0.005);
                extenderMotor.set(1);
            }

    }

    /*public void toHighBasket(float triggerPressure) {
        slideToPosition(EXTENSION_HIGH_BASKET, triggerPressure);
        rotateToHighBasket();
    }
*/
    public void toLowBasket(float triggerPressure) {
        slideToPosition(EXTENSION_LOW_BASKET, triggerPressure);
    }

    /*public void toBasePosition(float triggerPressure) {
        slideToPosition(EXTENSION_BASE, triggerPressure);
        rotateToBase();
    }
*/
    public boolean isAtBasePosition() {
        return isExtenderAtPosition(EXTENSION_BASE);
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
    }

public int getPosition(){
        return extenderMotor.getCurrentPosition();
}}
