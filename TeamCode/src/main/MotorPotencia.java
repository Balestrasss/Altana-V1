package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorPotencia", group = "Linear")
public class MotorPotencia extends LinearOpMode {

    private DcMotor motorPotencia;
    private Servo servoPotencia;
    private Servo servo2Potencia;

    private final double POSICAO_0_GRAUS = 0.0;
    private final double POSICAO_45_GRAUS = 0.25;

    @Override
    public void runOpMode() {
        engMotors();
        telemetry.addData("Status: ", "Pronto");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            motors();
            servo();
            servo2();

            telemetry.addData("motorPotencia Ativado", "%.2f", motorPotencia.getPower());
            telemetry.addData("Servo Potencia","%.2f", servoPotencia.getPosition());
            telemetry.addData("Servo2 Potencia","%.2f", servo2Potencia.getPosition());
            telemetry.update();
        }
    }
    public void motors() {
        if (gamepad1.right_trigger > 0.1) {
            motorPotencia.setPower(0.59);
        } else {
            motorPotencia.setPower(0.0);
        }
    }

    public void servo() {
        if(gamepad1.a){
            servoPotencia.setDirection(Servo.Direction.FORWARD);
            servoPotencia.setPosition(POSICAO_45_GRAUS);
        }
        if(gamepad1.x){
            servoPotencia.setDirection(Servo.Direction.REVERSE);
            servoPotencia.setPosition(POSICAO_0_GRAUS);
        }
    }

    public void servo2() {
        if(gamepad1.a){
            servo2Potencia.setDirection(Servo.Direction.FORWARD);
            servo2Potencia.setPosition(POSICAO_45_GRAUS);
        }
        if(gamepad1.x){
            servo2Potencia.setDirection(Servo.Direction.REVERSE);
            servo2Potencia.setPosition(POSICAO_0_GRAUS);
        }
    }



    public void engMotors() {
        motorPotencia = hardwareMap.get(DcMotor.class, "motorPotencia");
        motorPotencia.setDirection(DcMotor.Direction.FORWARD);

        servoPotencia = hardwareMap.get(Servo.class, "servoPotencia");
        servoPotencia.setDirection(Servo.Direction.FORWARD);

        servo2Potencia = hardwareMap.get(Servo.class, "servo2Potencia");
        servo2Potencia.setDirection(Servo.Direction.REVERSE);

        servoPotencia.setPosition(POSICAO_0_GRAUS);
        servo2Potencia.setPosition(POSICAO_0_GRAUS);


    }
}
