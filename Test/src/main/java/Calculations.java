import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;

public class Calculations {
    public double g=9.8015;
    public double SamplePeriod = 1.0/60;
    public Matrix SlowWalk1;
    public Matrix Error_Acc1;
    public Matrix Gyroscope;
    public Matrix Accelerometer;
    public double gama;
    public double seita;
    public double fai;
    public Matrix q;
    public MadgwickAHRS AHRS;
    public Matrix Acc_E;
    public Matrix Angle_V_E;
    public int t;

    public Calculations(Matrix Error,Matrix fristSlow){
        this.Error_Acc1 = Error;
        Matrix dq,deuler,Angle_V,Acc;
        dq = fristSlow.selectColumns(org.ujmp.core.calculation.Calculation.Ret.NEW,0,1,2,3);
        SlowWalk1 = DenseMatrix.Factory.zeros(1, 10);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,0),0,0);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,1),0,1);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,2),0,2);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,3),0,3);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,4),0,4);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,5),0,5);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,6),0,6);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,7),0,7);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,8),0,8);
        SlowWalk1.setAsDouble(fristSlow.getAsDouble(0,9),0,9);
        deuler = util.quatern2euler(dq);
        Angle_V = deuler.times(-1).divide(SamplePeriod);
        Gyroscope = DenseMatrix.Factory.zeros(1, 3);
        Gyroscope.setAsDouble(Angle_V.getAsDouble(0,0)*180/Math.PI,0,0);
        Gyroscope.setAsDouble(Angle_V.getAsDouble(0,1)*180/Math.PI,0,1);
        Gyroscope.setAsDouble(Angle_V.getAsDouble(0,2)*180/Math.PI,0,2);
        Acc=Error_Acc1
                .mtimes(
                        fristSlow
                                .selectColumns(org.ujmp.core.calculation.Calculation.Ret.NEW,4,5,6)
                                .transpose()
                                .divide(SamplePeriod)
                                .appendVertically(
                                        org.ujmp.core.calculation.Calculation.Ret.NEW,
                                        Matrix.Factory.ones(1,fristSlow.getRowCount()
                                        )
                                )
                );
        Accelerometer = DenseMatrix.Factory.zeros(1, 3);
        Accelerometer.setAsDouble(Acc.transpose().getAsDouble(0,0),0,0);
        Accelerometer.setAsDouble(Acc.transpose().getAsDouble(0,1),0,1);
        Accelerometer.setAsDouble(Acc.transpose().getAsDouble(0,2),0,2);
        t = 0;
        double Acc_X0 = Acc.getAsDouble(0,0);
        double Acc_Y0 = Acc.getAsDouble(1,0);
        double Acc_Z0 = Acc.getAsDouble(2,0);
        gama = Math.atan(Acc_X0/Acc_Z0);
        seita = -Math.asin(Acc_Y0);
        fai = 0;
        q = DenseMatrix.Factory.zeros(4, 1);
        q.setAsDouble(Math.cos(fai/2)*Math.cos(seita/2)*Math.cos(gama/2)-Math.sin(fai/2)*Math.sin(seita/2)*Math.sin(gama/2),0,0);
        q.setAsDouble(Math.cos(fai/2)*Math.sin(seita/2)*Math.cos(gama/2)-Math.sin(fai/2)*Math.cos(seita/2)*Math.sin(gama/2),1,0);
        q.setAsDouble(Math.sin(fai/2)*Math.sin(seita/2)*Math.cos(gama/2)+Math.cos(fai/2)*Math.cos(seita/2)*Math.sin(gama/2),2,0);
        q.setAsDouble(Math.sin(fai/2)*Math.cos(seita/2)*Math.cos(gama/2)+Math.cos(fai/2)*Math.sin(seita/2)*Math.sin(gama/2),3,0);
        AHRS = new MadgwickAHRS(1.0/60,q.transpose().toDoubleArray()[0], 0.1);
        updateFourNum();
    }

    public void updateFourNum(){
        Matrix quaternion = DenseMatrix.Factory.zeros(1, 4);
        AHRS.UpdateIMU(
                Gyroscope.selectRows(org.ujmp.core.calculation.Calculation.Ret.NEW,t).times(Math.PI/180).toDoubleArray()[0],
                Accelerometer.selectRows(org.ujmp.core.calculation.Calculation.Ret.NEW,t).toDoubleArray()[0]
        );
        for(int i = 0 ; i < 4 ; i ++){
            quaternion.setAsDouble(AHRS.Quaternion[i],0,i);
        }
        double qt[] = quaternion.toDoubleArray()[0];
        Matrix mem1 = DenseMatrix.Factory.zeros(1, 4);
        mem1.setAsDouble(0,0,0);
        mem1.setAsDouble(Accelerometer.getAsDouble(0,0),0,1);
        mem1.setAsDouble(Accelerometer.getAsDouble(0,1),0,2);
        mem1.setAsDouble(Accelerometer.getAsDouble(0,2),0,3);
        Matrix mem2 = DenseMatrix.Factory.zeros(1, 4);
        mem2.setAsDouble(0,0,0);
        mem2.setAsDouble(Gyroscope.getAsDouble(0,0),0,1);
        mem2.setAsDouble(Gyroscope.getAsDouble(0,1),0,2);
        mem2.setAsDouble(Gyroscope.getAsDouble(0,2),0,3);
        Matrix tem1,tem2;
        tem1 = util.quaternProd(Matrix.Factory.importFromArray(qt),util.quaternProd(mem1,util.quaternConj(Matrix.Factory.importFromArray(qt))));
        tem2 = util.quaternProd(Matrix.Factory.importFromArray(qt),util.quaternProd(mem2,util.quaternConj(Matrix.Factory.importFromArray(qt))));
        if(t != 0) {
            Acc_E = Acc_E.appendHorizontally(org.ujmp.core.calculation.Calculation.Ret.NEW, tem1.transpose());
            Angle_V_E = Angle_V_E.appendHorizontally(org.ujmp.core.calculation.Calculation.Ret.NEW, tem2.transpose());
        }
        else{
            Acc_E = DenseMatrix.Factory.zeros(4, 1);
            Angle_V_E = DenseMatrix.Factory.zeros(4, 1);
            Acc_E.setAsDouble(tem1.getAsDouble(0,0),0,0);
            Acc_E.setAsDouble(tem1.getAsDouble(0,1),1,0);
            Acc_E.setAsDouble(tem1.getAsDouble(0,2),2,0);
            Acc_E.setAsDouble(tem1.getAsDouble(0,3),3,0);
            Angle_V_E.setAsDouble(tem2.getAsDouble(0,0),0,0);
            Angle_V_E.setAsDouble(tem2.getAsDouble(0,1),1,0);
            Angle_V_E.setAsDouble(tem2.getAsDouble(0,2),2,0);
            Angle_V_E.setAsDouble(tem2.getAsDouble(0,3),3,0);
        }
    }

    public void updateSlowWalk(Matrix newSlow){
        Matrix dq,deuler,Angle_V,Acc;
        dq = newSlow.selectColumns(org.ujmp.core.calculation.Calculation.Ret.NEW,0,1,2,3);
        SlowWalk1 = SlowWalk1.appendVertically(org.ujmp.core.calculation.Calculation.Ret.NEW,newSlow);
        deuler = util.quatern2euler(dq);
        Angle_V = deuler.times(-1).divide(SamplePeriod);
        Gyroscope = Gyroscope.appendVertically(org.ujmp.core.calculation.Calculation.Ret.NEW,Angle_V).times(180/Math.PI);;
        Acc=Error_Acc1
                .mtimes(
                        newSlow
                                .selectColumns(org.ujmp.core.calculation.Calculation.Ret.NEW,4,5,6)
                                .transpose()
                                .divide(SamplePeriod)
                                .appendVertically(
                                        org.ujmp.core.calculation.Calculation.Ret.NEW,
                                        Matrix.Factory.ones(1,newSlow.getRowCount()
                                        )
                                )
                );
        Accelerometer = Accelerometer.appendVertically(org.ujmp.core.calculation.Calculation.Ret.NEW,Acc.transpose());
        t ++;
        updateFourNum();
    }

    public void runOnce() {
//        Matrix dq,deuler,Angle_V,Acc;
//        dq = SlowWalk1.selectColumns(SlowWalk1.NEW,0,1,2,3);
//        deuler = util.quatern2euler(dq);
//        Angle_V = deuler.times(-1).divide(SamplePeriod);
//        Acc=Error_Acc1
//                .mtimes(
//                    SlowWalk1
//                            .selectColumns(SlowWalk1.NEW,4,5,6)
//                            .transpose()
//                            .divide(SamplePeriod)
//                            .appendVertically(
//                                    SlowWalk1.NEW,
//                                    Matrix.Factory.ones(1,SlowWalk1.getRowCount()
//                                    )
//                            )
//                );
//        Matrix Accelerometer=Acc.transpose();
//        Matrix Gyroscope=Angle_V.clone();
//        double A = 0,B = SamplePeriod,C = SamplePeriod*(SlowWalk1.getRowCount()-1);
//        int n = (int)((C-A)/B)+1;
//        Matrix time=DenseMatrix.Factory.zeros(1, n);
//        time.setAsDouble(A,0,0);
//        time.setAsDouble(A,0,n-1);
//        time.setAsDouble(C,0,n-1);
//        for(int i = 1; i < n -1 ; i ++){
//            time.setAsDouble(A + B * i,0,i) ;
//        }
//        Gyroscope=Gyroscope.times(180/Math.PI);
//        double Acc_X0 = Acc.getAsDouble(0,0);
//        double Acc_Y0 = Acc.getAsDouble(1,0);
//        double Acc_Z0 = Acc.getAsDouble(2,0);
//        double gama = Math.atan(Acc_X0/Acc_Z0);
//        double seita = -Math.asin(Acc_Y0);
//        double fai = 0;
//        Matrix q = DenseMatrix.Factory.zeros(4, 1);
//        q.setAsDouble(Math.cos(fai/2)*Math.cos(seita/2)*Math.cos(gama/2)-Math.sin(fai/2)*Math.sin(seita/2)*Math.sin(gama/2),0,0);
//        q.setAsDouble(Math.cos(fai/2)*Math.sin(seita/2)*Math.cos(gama/2)-Math.sin(fai/2)*Math.cos(seita/2)*Math.sin(gama/2),1,0);
//        q.setAsDouble(Math.sin(fai/2)*Math.sin(seita/2)*Math.cos(gama/2)+Math.cos(fai/2)*Math.cos(seita/2)*Math.sin(gama/2),2,0);
//        q.setAsDouble(Math.sin(fai/2)*Math.cos(seita/2)*Math.cos(gama/2)+Math.cos(fai/2)*Math.sin(seita/2)*Math.sin(gama/2),3,0);
//        MadgwickAHRS AHRS = new MadgwickAHRS(1.0/60,q.transpose().toDoubleArray()[0], 0.1);
//        Matrix quaternion = DenseMatrix.Factory.zeros(time.getColumnCount(), 4);
//        for(int t = 0 ; t < time.getColumnCount() ; t ++){
//            AHRS.UpdateIMU(
//                    Gyroscope.selectRows(Gyroscope.NEW,t).times(Math.PI/180).toDoubleArray()[0],
//                    Accelerometer.selectRows(Accelerometer.NEW,t).toDoubleArray()[0]
//                    );
//            for(int i = 0 ; i < 4 ; i ++){
//                quaternion.setAsDouble(AHRS.Quaternion[i],t,i);
//            }
//        }
//        Matrix Acc_E,Angle_V_E,Mag_E;
//        Acc_E = DenseMatrix.Factory.zeros(3, quaternion.getRowCount());
//        Angle_V_E = DenseMatrix.Factory.zeros(3, quaternion.getRowCount());
//        for(int i = 0; i < quaternion.getRowCount(); i ++){
//            double qt[] = quaternion.selectRows(quaternion.NEW,i).toDoubleArray()[0];
//            Matrix mem1 = DenseMatrix.Factory.zeros(1, 4);
//            mem1.setAsDouble(0,i,0);
//            mem1.setAsDouble(Accelerometer.getAsDouble(i,0),i,1);
//            mem1.setAsDouble(Accelerometer.getAsDouble(i,1),i,2);
//            mem1.setAsDouble(Accelerometer.getAsDouble(i,2),i,3);
//            Matrix mem2 = DenseMatrix.Factory.zeros(1, 4);
//            mem2.setAsDouble(0,i,0);
//            mem2.setAsDouble(Gyroscope.getAsDouble(i,0),i,1);
//            mem2.setAsDouble(Gyroscope.getAsDouble(i,1),i,2);
//            mem2.setAsDouble(Gyroscope.getAsDouble(i,2),i,3);
//            Matrix tem1,tem2;
//            tem1 = util.quaternProd(Matrix.Factory.importFromArray(qt),util.quaternProd(mem1,util.quaternConj(Matrix.Factory.importFromArray(qt))));
//            tem2 = util.quaternProd(Matrix.Factory.importFromArray(qt),util.quaternProd(mem2,util.quaternConj(Matrix.Factory.importFromArray(qt))));
//            Acc_E.setAsDouble(tem1.getAsDouble(0,0),0,i);
//            Acc_E.setAsDouble(tem1.getAsDouble(0,1),1,i);
//            Acc_E.setAsDouble(tem1.getAsDouble(0,2),2,i);
//            Acc_E.setAsDouble(tem1.getAsDouble(0,3),3,i);
//            Angle_V_E.setAsDouble(tem2.getAsDouble(0,0),0,i);
//            Angle_V_E.setAsDouble(tem2.getAsDouble(0,1),1,i);
//            Angle_V_E.setAsDouble(tem2.getAsDouble(0,2),2,i);
//            Angle_V_E.setAsDouble(tem2.getAsDouble(0,3),3,i);
//        }
//        Matrix Ve = DenseMatrix.Factory.zeros( time.getColumnCount() + 1, 3);
//        Matrix Postion_E = jifen(Ve,1,1600);
    }

    public Matrix jifen(Matrix Ve, int t1, int t2){
        Matrix Postion_E = DenseMatrix.Factory.zeros( t2 - t1 + 1, 3);
        for( int t = t1; t <= t2 ; t ++){
            Ve.setAsDouble(Ve.getAsDouble((long)(t + 1 - t1) ,0)+Acc_E.getAsDouble(1,t ) * SamplePeriod,t + 2 - t1,0);
            Ve.setAsDouble(Ve.getAsDouble((long)(t + 1 - t1) ,1)+Acc_E.getAsDouble(2,t ) * SamplePeriod,t + 2 - t1,1);
            Ve.setAsDouble(Ve.getAsDouble((long)(t + 1 - t1) ,2)+(Acc_E.getAsDouble(3,t )-g) * SamplePeriod,t + 2 - t1,2);
        }
        System.out.println("Ve");
        System.out.println(Ve);
        for( int t = t1; t <= t2 ; t ++){
            Postion_E.setAsDouble(Postion_E.getAsDouble(t + 1 - t1, 0) + Ve.getAsDouble(t + 2 - t1, 0) * SamplePeriod, t + 2 - t1,0);
            Postion_E.setAsDouble(Postion_E.getAsDouble(t + 1 - t1, 1) + Ve.getAsDouble(t + 2 - t1, 1) * SamplePeriod, t + 2 - t1,1);
            Postion_E.setAsDouble(Postion_E.getAsDouble(t + 1 - t1, 2) + Ve.getAsDouble(t + 2 - t1, 2) * SamplePeriod, t + 2 - t1,2);
        }
        return  Postion_E;
    }

    public int findT1() {
        int ret = 1;
        return ret;
    }

    public int findT2() {
        int ret = 1;
        return ret;
    }

}
