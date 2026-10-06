package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver; //Biblioteca da GoBilda


@TeleOp(name = "MainCode", group = "Linear")
public class MainCode extends LinearOpMode {

    // Locomoção
    private DcMotor Ldf, Ldt, Lef, Let;

    // Coletor
    private DcMotor Coletor;
    private boolean coletorLigado  = false;
    private boolean botaoAAnterior = false;

    // Servo direcionador
    private CRServo ServoC;
    private boolean servoLigado = false;
    private boolean botaoXControle2 = false;

    //servos pra flor mas q vai ligar junto do coletor
    private CRServo FlorR;
    private CRServo FlorL;
    private boolean servoRLigado = false;
    private boolean servoLLigado = false;

    @Override
    public void runOpMode() {
        variaveis();
        invmotores();

        telemetry.addData("Status", "Pronto");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            movimentacao();
            coletor();
            telemetriafinal();
            variaveis();
            servoC();

        }

    }

    public void invmotores() {
        Lef.setDirection(DcMotor.Direction.REVERSE);
        Let.setDirection(DcMotor.Direction.REVERSE);
        Ldf.setDirection(DcMotor.Direction.FORWARD);
        Ldt.setDirection(DcMotor.Direction.FORWARD);
        Coletor.setDirection(DcMotor.Direction.FORWARD);

    }


    // Controle 1 - Locomoção
    public void movimentacao() {
        double  axial   = -this.gamepad1.left_stick_y,
                lateral =  this.gamepad1.left_stick_x,
                rot     =  this.gamepad1.right_stick_x;

        double VLef = axial - lateral + rot,
                VLdf = axial + lateral - rot,
                VLet = axial + lateral + rot,
                VLdt = axial - lateral - rot;

        double max = Math.max(Math.abs(VLef), Math.abs(VLdf));
        max = Math.max(max, Math.abs(VLet));
        max = Math.max(max, Math.abs(VLdt));

        if (max > 1) {
            VLef /= max;
            VLdf /= max;
            VLet /= max;
            VLdt /= max;
        }

        Lef.setPower(VLef);
        Ldf.setPower(VLdf);
        Let.setPower(VLet);
        Ldt.setPower(VLdt);
    }

    // Controle 1 - Coletor (botão A)
    public void coletor() {
        boolean botaoAAtual = this.gamepad1.a;

        if (botaoAAtual && !botaoAAnterior) {
            coletorLigado = !coletorLigado;
            servoRLigado  = !servoRLigado;
            servoLLigado  = !servoLLigado;

        }

        Coletor.setPower(coletorLigado ? 1.0 : 0.0);
        FlorR.setPower(1.0);
        FlorL.setPower(1.0);

        botaoAAnterior = botaoAAtual;
    }

    public void servoC(){
        boolean botaoXControle2 = this.gamepad2.x;

        if (botaoXControle2) {
            servoLigado = !servoLigado;

        }

        if (servoLigado){
            ServoC.setPower(1.0);
        }
        else {
            ServoC.setPower(0.0);
        }
    }

    }


    public void variaveis() {
        Ldf     = hardwareMap.get(DcMotor.class,   "frontRight");
        Ldt     = hardwareMap.get(DcMotor.class,   "backRight");
        Lef     = hardwareMap.get(DcMotor.class,   "frontLeft");
        Let     = hardwareMap.get(DcMotor.class,   "backLeft");
        Coletor = hardwareMap.get(DcMotor.class,   "motorExtra");
        ServoC  = hardwareMap.get(CRSerco.class,   "ServoColetor");
        FLorR   = hardwareMap.get(CRServo.class,   "ServoFlorDireita");
        FLorL   = hardwareMap.get(CRServo.class,   "ServoFlorEsquerda");


    }

    public void telemetriafinal() {
        telemetry.addData("Motor Lef",  "%.2f", Lef.getPower());
        telemetry.addData("Motor Ldf",  "%.2f", Ldf.getPower());
        telemetry.addData("Motor Let",  "%.2f", Let.getPower());
        telemetry.addData("Motor Ldt",  "%.2f", Ldt.getPower());
        telemetry.addData("Coletor",    coletorLigado ? "LIGADO" : "DESLIGADO");
        telemetry.addData("Servo Flores", FlorR && FlorL ? "LIGADOS" : "DESLIGAODS");
        telemetry.update();
    }

