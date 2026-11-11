package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BIOBUZZ - Autonomo POR TEMPO (sem odometria).
 *
 * Cada movimento liga as rodas com potencia fixa por um tempo e desliga.
 * O tempo e calculado pela distancia/angulo pedido usando as velocidades
 * da secao 2 (que voces medem uma vez com trena).
 */
@Autonomous(name = "AutoTestBF")
public class AutoTestBF extends LinearOpMode {

    // =====================================================================
    //  1) ESCREVA O SEU AUTONOMO AQUI
    // =====================================================================
    //  MOVIMENTO (distancia em mm, angulo em graus):
    //    irFrente(mm)      irTras(mm)      irEsquerda(mm)      irDireita(mm)
    //    girarEsq(graus)   girarDir(graus)                     -> 90 = um quarto de volta
    //    parar(segundos)   -> fica parado esse tempo
    //
    //  MECANISMOS:
    //    lancar()          -> liga o lancador (motorPotencia)
    //    atirar(segundos)  -> gira o gatilho (ServoColetor) esse tempo
    //    coletar()         -> sequencia do coletor (leva ~6 s)
    //    hood()            -> modelador em 0.5      hoodFechar() -> modelador em 0.0
    //    desligarMecanismos()
    //
    //  EXEMPLO:
    //    lancar(); parar(1.2); atirar(2);
    //    irFrente(600); girarEsq(90); irFrente(400); girarDir(90); irTras(200);
    // =====================================================================
    public void trajeto() {
        irFrente(500);
        girarEsq(120);
        irFrente(500);
        girarDir(120);
        irFrente(700);
    }

    // =====================================================================
    //  2) AJUSTES (MEDIR UMA VEZ)
    // =====================================================================
    static final double POT_RETA = 0.40;   // potencia andando
    static final double POT_GIRO = 0.35;   // potencia girando

    // Quanto o robo anda/gira por SEGUNDO nessas potencias. COMO MEDIR:
    //  - deixe so irFrente(1000) no trajeto, rode e meca com trena quanto andou (ex.: 800 mm)
    //    -> MM_POR_SEG_FRENTE = MM_POR_SEG_FRENTE * 800 / 1000
    //  - mesma coisa com irDireita(1000) para MM_POR_SEG_LADO
    //  - so girarEsq(360) e veja quanto girou (ex.: 300 graus) para GRAUS_POR_SEG
    static final double MM_POR_SEG_FRENTE = 600;   // TODO medir
    static final double MM_POR_SEG_LADO   = 500;   // TODO medir (de lado o mecanum anda menos)
    static final double GRAUS_POR_SEG     = 180;   // TODO medir

    static final double PAUSA = 0.15;   // s parado entre um movimento e outro (robo assentar)

    // =====================================================================
    //  3) FUNCOES DE MOVIMENTO
    // =====================================================================
    public void irFrente(double mm)   { andarTempo( POT_RETA, 0, 0, mm / MM_POR_SEG_FRENTE); }
    public void irTras(double mm)     { andarTempo(-POT_RETA, 0, 0, mm / MM_POR_SEG_FRENTE); }
    public void irDireita(double mm)  { andarTempo(0,  POT_RETA, 0, mm / MM_POR_SEG_LADO); }
    public void irEsquerda(double mm) { andarTempo(0, -POT_RETA, 0, mm / MM_POR_SEG_LADO); }

    public void girarDir(double graus) { andarTempo(0, 0,  POT_GIRO, graus / GRAUS_POR_SEG); }
    public void girarEsq(double graus) { andarTempo(0, 0, -POT_GIRO, graus / GRAUS_POR_SEG); }

    public void parar(double segundos) {
        parar();
        esperar(segundos);
    }

    /** Liga as rodas (axial, lateral, rot) por 'segundos' e para. */
    private void andarTempo(double axial, double lateral, double rot, double segundos) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < segundos) {
            mover(axial, lateral, rot);
            telemetry.addData("Movimento", "%.1f de %.1f s", t.seconds(), segundos);
            telemetry.update();
        }
        parar();
        esperar(PAUSA);
    }

    // =====================================================================
    //  4) MECANISMOS
    // =====================================================================
    public void lancar() {
        Mt.setPower(0.529);
    }

    public void atirar(double segundos) {
        Sg.setPower(1.0);
        esperar(segundos);
        Sg.setPower(0);
    }

    public void coletar() {
        Mc2.setPower(1.0);
        esperar(2);
        Mc.setPower(1.0);
        Sg.setPower(1.0);
        esperar(4.0);
        Sg.setPower(0);
    }

    public void hood() {
        servoPotencia.setPosition(0.5);
        servo2Potencia.setPosition(0.5);
    }

    public void hoodFechar() {
        servoPotencia.setPosition(0.0);
        servo2Potencia.setPosition(0.0);
    }

    public void desligarMecanismos() {
        Mt.setPower(0);
        Mc.setPower(0);
        Mc2.setPower(0);
        Sg.setPower(0);
    }

    // =====================================================================
    //  5) BASE (nao precisa mexer)
    // =====================================================================
    private DcMotor Lef, Ldf, Let, Ldt, Mt, Mc, Mc2;
    private CRServo Sg;
    private Servo servoPotencia, servo2Potencia;

    @Override
    public void runOpMode() {
        config();

        telemetry.addData("Status", "Pronto");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        trajeto();

        parar();
        desligarMecanismos();
    }

    private void config() {
        Lef = hardwareMap.get(DcMotor.class, "frontLeft");
        Ldf = hardwareMap.get(DcMotor.class, "frontRight");
        Let = hardwareMap.get(DcMotor.class, "backLeft");
        Ldt = hardwareMap.get(DcMotor.class, "backRight");

        Lef.setDirection(DcMotor.Direction.REVERSE);
        Ldf.setDirection(DcMotor.Direction.FORWARD);
        Let.setDirection(DcMotor.Direction.REVERSE);
        Ldt.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{Lef, Ldf, Let, Ldt}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Mt = hardwareMap.get(DcMotor.class, "motorPotencia");
        Mt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Mc  = hardwareMap.get(DcMotor.class, "motorExtra");
        Mc2 = hardwareMap.get(DcMotor.class, "motorAuxiliar");
        Mc2.setDirection(DcMotor.Direction.REVERSE);

        Sg = hardwareMap.get(CRServo.class, "ServoColetor");
        Sg.setDirection(DcMotorSimple.Direction.FORWARD);

        servoPotencia  = hardwareMap.get(Servo.class, "servoPotencia");
        servo2Potencia = hardwareMap.get(Servo.class, "servo2Potencia");
        servoPotencia.setDirection(Servo.Direction.FORWARD);
        servo2Potencia.setDirection(Servo.Direction.REVERSE);
    }

    /** axial + = frente | lateral + = direita | rot + = girar p/ direita */
    private void mover(double axial, double lateral, double rot) {
        double vLef = axial - lateral + rot;
        double vLdf = axial + lateral - rot;
        double vLet = axial + lateral + rot;
        double vLdt = axial - lateral - rot;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(vLef), Math.abs(vLdf)),
                Math.max(Math.abs(vLet), Math.abs(vLdt))));
        Lef.setPower(vLef / max);
        Ldf.setPower(vLdf / max);
        Let.setPower(vLet / max);
        Ldt.setPower(vLdt / max);
    }

    private void parar() {
        mover(0, 0, 0);
    }

    /** sleep que respeita o STOP do arbitro. */
    private void esperar(double s) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < s) idle();
    }
}
