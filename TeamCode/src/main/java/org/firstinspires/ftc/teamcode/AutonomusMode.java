package org.firstinspires.ftc.robotcontroller;

import static org.firstinspires.ftc.robotcontroller.OdoTreino.DIR_ENC_X;
import static org.firstinspires.ftc.robotcontroller.OdoTreino.DIR_ENC_Y;
import static org.firstinspires.ftc.robotcontroller.OdoTreino.START_H_DEG;
import static org.firstinspires.ftc.robotcontroller.OdoTreino.START_X_MM;
import static org.firstinspires.ftc.robotcontroller.OdoTreino.START_Y_MM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class fds extends LinearOpMode {
    //NOMES NO "CONFIGURE ROBOT" (Driver Station)
    static final String M_LEF = "frontleft";
    static final String M_LDF = "frontright";
    static final String M_LET = "backleft";
    static final String M_LDT = "backright";
    static final String PINPOINT = "odo";
    static final String CAMERA = "Webcam 1";
    static final String INTAKE = "intake";
    static final String LAUNCHER = "launcher";
    static final String S_YAW_ESQ = "yawEsq";
    static final String S_YAW_DIR = "yawDir";
    static final String S_PITCH = "pitch";

    //Variaveis do robo
    static final double xOffset = 95;
    static final double yOffset = 100;
    static final double tol = 10;
    static final double tolH = 5;
    private double pot = 0.5;
    private double x, y, h;
    private double erX, erY, erH;

    //hardware
    private DcMotor Lef, Ldf, Let, Ldt, intake;
    private GoBildaPinpointDriver odo;

    //config motores e servos
    public void declarar(){
        Lef = hardwareMap.get(DcMotor.class, M_LEF);
        Let = hardwareMap.get(DcMotor.class, M_LET);
        Ldf = hardwareMap.get(DcMotor.class, M_LDF);
        Ldt = hardwareMap.get(DcMotor.class, M_LDT);

        intake = hardwareMap.get(DcMotor.class, "intake");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);

    }
    public void config(){
        //Motores
        Lef.setDirection(DcMotorSimple.Direction.FORWARD);
        Let.setDirection(DcMotorSimple.Direction.REVERSE);
        Ldf.setDirection(DcMotorSimple.Direction.REVERSE);
        Ldt.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo

        //Odo
        odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(DIR_ENC_X, DIR_ENC_Y);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.MM, START_X_MM, START_Y_MM, AngleUnit.DEGREES, START_H_DEG));

    }
    public void atualizarOdo(){
        odo.update();
        Pose2D pos = odo.getPosition();
        x = pos.getX(DistanceUnit.MM);
        y = pos.getY(DistanceUnit.MM);
        h = pos.getHeading(AngleUnit.DEGREES);
    }
    //Correção do movimento(Ponto que precisa ver dps)

    public void corMov (double alvoX, double alvoY, double alvoH){
        erX = alvoX - x;
        erY = alvoY - y;
        erH = alvoH - h;

        //Correção angulo
        while(erH > 180) erH -= 360;
        while(erH < -180) erH += 360;

        if (erH > tolH) giroEsq();
        else if (erH < -tolH) giroDir();

            //Correção movimento
        else if (erX > tol) movFre();
        else if (erX < -tol) movTra();

        else if (erY > tol) movEsq();
        else if (erY < -tol) movDir();
        else parar();
    }

    //Movimentação (Ponto que precisa ver dps)
    public void parar(){
        Lef.setPower(0);
        Let.setPower(0);
        Ldt.setPower(0);
        Ldf.setPower(0);
    }
    public void movFre(){
        Lef.setPower(pot);
        Ldf.setPower(pot);
        Let.setPower(pot);
        Ldt.setPower(pot);
    }
    public void movTra(){
        Lef.setPower(-pot);
        Ldf.setPower(-pot);
        Let.setPower(-pot);
        Ldt.setPower(-pot);
    }
    public void movEsq() {
        Lef.setPower(-pot);
        Ldf.setPower(pot);
        Let.setPower(pot);
        Ldt.setPower(-pot);
    }
    public void movDir() {
        Lef.setPower(pot);
        Ldf.setPower(-pot);
        Let.setPower(-pot);
        Ldt.setPower(pot);
    }
    public void giroEsq() {
        Lef.setPower(-pot);
        Ldf.setPower(pot);
        Let.setPower(-pot);
        Ldt.setPower(pot);
    }
    public void giroDir() {
        Lef.setPower(pot);
        Ldf.setPower(-pot);
        Let.setPower(pot);
        Ldt.setPower(-pot);
    }

    //Telemetry
    public void tel(){
        telemetry.addData("Erro X", erX);
        telemetry.addData("Erro Y", erY);
        telemetry.addData("Erro Angulo", erH);
        telemetry.update();
    }

    //Trajeto percorrido (Exemplo)
    public boolean chegou(double alvoX, double alvoY, double alvoH){
        double eH = alvoH - h;
        while(eH > 180) eH -= 360;
        while(eH < -180) eH += 360;

        boolean b = Math.abs(alvoX - x) <= tol && Math.abs(alvoY - y) <= tol && Math.abs(eH) <= tolH;
        return b;
    }
    public void trajeto(){
        while(opModeIsActive() && !chegou(1500, 0, 0)) {
            corMov(1500, 0, 0);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(0, 0, 90)) {
            corMov(0, 0, 90);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(0, 1500, 0)) {
            corMov(0, 1500, 0);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(0, 0, 90)) {
            corMov(0, 0, 90);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(1500, 0, 0)) {
            corMov(1500, 0, 0);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(0, 0, 90)) {
            corMov(0, 0, 90);
            atualizarOdo();
            sleep(20);
        }
        while(opModeIsActive() && !chegou(0, 1500, 0)) {
            corMov(0, 1500, 0);
            atualizarOdo();
            sleep(20);
        }

    }

    @Override
    public void runOpMode(){
        declarar();
        config();
        waitForStart();
        trajeto();

        while(opModeIsActive()){
            atualizarOdo();
            tel();
        }
    }
}
