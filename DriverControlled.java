package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver Controlled", group="OpMode")
public class DriverControlled extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    //movement variables
    private double x, y, r, speed = robot.DEFAULT_SPEED, speed_threshold = 0.25;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, false);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        drive();

        outputData();
    }

    public void drive(){
        if (gamepad1.right_trigger > speed_threshold)
            speed = gamepad1.right_trigger;
        else
            speed = speed_threshold;

        x = gamepad1.right_stick_x;
        y = gamepad1.right_stick_y;
        r = gamepad1.left_stick_x;

        robot.move(x,y,r,speed);
    }

    public void outputData(){
        telemetry.addData("movement(x,y,r): ", x + ", " + y + ", " + r);
        telemetry.update();
    }

}
