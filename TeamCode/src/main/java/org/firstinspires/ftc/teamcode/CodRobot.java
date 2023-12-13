package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="CodRobot")
public class CodRobot extends OpMode {

    private DcMotorEx frontLeft  = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft   = null;
    private DcMotorEx backRight  = null;

    private DcMotorEx motorKatanaDreapta = null;
    private DcMotorEx motorKatanaStanga = null;
    private DcMotor motorColector = null;

    private Servo servoGhearaStanga = null;
    private Servo servoGhearaDreapta = null;

    private DistanceSensor sensorRange;
    int pozitie;

    private BNO055IMU imu;

    @Override
    public void init() {
        hardwareMap();
    }

    @Override
    public void loop() {
        miscare();
        executare();
    }


    void executare(){

        //if(gheara()==1){
        if (gamepad2.y)
            pozitie = 4100;
        else if (gamepad2.b)
            pozitie = 2900;
        else if (gamepad2.a)
            pozitie = 1800;
        else if(gamepad2.x) {
            pozitie = 0;
            deschidereGheara();
        }

        if(motorKatanaStanga.getCurrentPosition()<4000 && motorKatanaDreapta.getCurrentPosition()<4000
                && gamepad2.right_trigger!=0 &&
                (motorKatanaStanga.getCurrentPosition()-motorKatanaStanga.getTargetPosition()<10
                        && motorKatanaDreapta.getCurrentPosition()-motorKatanaDreapta.getTargetPosition()>-10
                        || motorKatanaStanga.getCurrentPosition()-motorKatanaStanga.getTargetPosition()<10 &&
                        motorKatanaDreapta.getCurrentPosition()-motorKatanaDreapta.getTargetPosition()>-10))
            pozitie=pozitie+250;
        else if(gamepad2.left_trigger!=0 && motorKatanaStanga.getCurrentPosition()>300
                && motorKatanaDreapta.getCurrentPosition()>300 &&
                (motorKatanaStanga.getCurrentPosition()-motorKatanaStanga.getTargetPosition()<10 &&
                        motorKatanaDreapta.getCurrentPosition()-motorKatanaDreapta.getTargetPosition()<10
                        || motorKatanaStanga.getCurrentPosition()-motorKatanaStanga.getTargetPosition()>-10 &&
                        motorKatanaDreapta.getCurrentPosition()-motorKatanaDreapta.getTargetPosition()>-10))
            pozitie=pozitie-250;

        encoder(motorKatanaDreapta, pozitie, 2000);
        encoder(motorKatanaStanga, pozitie, 2000);

        if(gamepad2.dpad_up)
            deschidereGheara();
        else if(gamepad2.dpad_down)
            inchidereGheara();


        if(gamepad2.right_bumper){
            motorColector.setPower(-1);
        }else{
            motorColector.setPower(0);
        }

        if(sensorRange.getDistance(DistanceUnit.CM)<12 && !motorKatanaDreapta.isBusy() && !motorKatanaStanga.isBusy() && motorKatanaDreapta.getCurrentPosition()<5 && motorKatanaStanga.getCurrentPosition()<5 && !gamepad2.dpad_up)
            inchidereGheara();
    }

    void encoder(DcMotorEx motor,int pozitie, int viteza){

        motor.setTargetPosition(pozitie);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(viteza);
    }

    void inchidereGheara(){//.
        servoGhearaStanga.setPosition(0);
        servoGhearaDreapta.setPosition(0.2);
    }
    void deschidereGheara(){
        servoGhearaStanga.setPosition(0.5);
        servoGhearaDreapta.setPosition(0.0);
    }
    int gheara(){
        if(servoGhearaDreapta.getPosition()!=0 && servoGhearaStanga.getPosition()==0.5)
            return 1;
        else
            return 0;
    }

    void miscare(){

        double fataSpate = -gamepad1.right_stick_y;
        double stangaDreapta = gamepad1.right_stick_x;
        double rotatie = -(gamepad1.right_trigger-gamepad1.left_trigger);
        double viteza = 1;
        if(gamepad1.left_bumper)
            viteza =2;

        double fl = Range.clip(fataSpate - stangaDreapta - rotatie,-1,1);
        double fr = Range.clip(fataSpate + stangaDreapta + rotatie,-1,1);
        double bl = Range.clip(fataSpate + stangaDreapta - rotatie,-1,1);
        double br = Range.clip(fataSpate - stangaDreapta + rotatie,-1,1);

        frontLeft.setPower(fr*fr*fr/viteza);
        frontRight.setPower(fl*fl*fl/viteza);
        backLeft.setPower(br*br*br/viteza);
        backRight.setPower(bl*bl*bl/viteza);

        if (gamepad1.dpad_up) {
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
        }else if (gamepad1.dpad_down) {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(-1);
        }else if (gamepad1.dpad_left) {
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(-1);
        }else if (gamepad1.dpad_right) {
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
        }

        if(gamepad1.y)
            Turn(90);
        else if(gamepad1.b)
            Turn(0);
        else if(gamepad1.a)
            Turn(-90);
        else if(gamepad1.x)
            Turn(180);
    }

    void hardwareMap(){

        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        frontLeft = hardwareMap.get(DcMotorEx.class,"fl");
        frontRight = hardwareMap.get(DcMotorEx.class,"fr");
        backRight = hardwareMap.get(DcMotorEx.class,"br");

        motorKatanaDreapta  = hardwareMap.get(DcMotorEx.class, "motorKatanaDreapta");
        motorKatanaStanga  = hardwareMap.get(DcMotorEx.class, "motorKatanaStanga");
        motorColector  = hardwareMap.get(DcMotorEx.class, "colector");

        servoGhearaStanga = hardwareMap.get(Servo.class, "ghearaStanga");
        servoGhearaDreapta = hardwareMap.get(Servo.class, "ghearaDreapta");

        motorKatanaDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorKatanaStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        motorKatanaDreapta.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        sensorRange = hardwareMap.get(DistanceSensor.class, "dis");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    void Turn(double targetAngle){

        TurnPIDController pid = new TurnPIDController(targetAngle, 0.1,0,0);

        while(Math.abs(targetAngle - PozitiaActuala())>1){

            double motorPower = pid.update(PozitiaActuala());

            frontRight.setPower(-motorPower);
            frontLeft.setPower(motorPower);
            backRight.setPower(-motorPower);
            backLeft.setPower(motorPower);
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    public double PozitiaActuala(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}