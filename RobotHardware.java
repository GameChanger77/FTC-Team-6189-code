package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //robot components
    public DcMotor motor1,motor2,motor3,motor4; //starts with left front and moves clockwise
    public DcMotor verticalLeft, verticalRight, horizontal; //Odometry motors

    public Servo claw;

    public BNO055IMU imu;
    public ColorSensor color;
    public DistanceSensor depth;

    //movement variables
    public final double DEFAULT_SPEED = 0.5;
    private double p1,p2,p3,p4;

    public void init(HardwareMap hardwareMap, boolean encoders){
        //motor configuration
        setHardwareMap(hardwareMap);
        setMotorDirections();
        setMotorZeroPowerBehavior();
        initEncoderMotors(hardwareMap);

        //encoder options
        if (encoders){
            resetEncoders();
        } else {
            noEncoders();
        }

        //gyro
        initGyro(hardwareMap);

    }

    public void setHardwareMap(HardwareMap hardwareMap){
        motor1 = hardwareMap.dcMotor.get("Motor1");
        motor2 = hardwareMap.dcMotor.get("Motor2");
        motor3 = hardwareMap.dcMotor.get("Motor3");
        motor4 = hardwareMap.dcMotor.get("Motor4");

        verticalLeft = hardwareMap.dcMotor.get("Motor3");
        verticalRight = hardwareMap.dcMotor.get("Motor1");
        horizontal = hardwareMap.dcMotor.get("Motor2");

        claw = hardwareMap.servo.get("claw");
        color = hardwareMap.colorSensor.get("color_sensor");
        depth = hardwareMap.get(DistanceSensor.class, "depth");
    }

    public void setMotorDirections(){
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotorZeroPowerBehavior(){
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initGyro(HardwareMap hardwareMap){
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

    public void initEncoderMotors(HardwareMap hardwareMap){
        verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void noEncoders() {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void useEncoders() {
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        useEncoders();
    }

    public void move(double x, double y, double r, double speed){
        p1 = x + y + r;
        p2 = x - y - r;
        p3 = x - y + r;
        p4 = x + y - r;

        motor1.setPower(p1 * speed);
        motor2.setPower(p1 * speed);
        motor3.setPower(p1 * speed);
        motor4.setPower(p1 * speed);
    }

    public void move(double x, double y, double r) {
        move(x,y,r,DEFAULT_SPEED);
    }

    public double[] getMovementPower(){
        double[] motorPower = new double[4];
        motorPower[0] = motor1.getPower();
        motorPower[1] = motor2.getPower();
        motorPower[2] = motor3.getPower();
        motorPower[3] = motor4.getPower();
        return motorPower;
    }

}
