import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;

import java.util.jar.JarEntry;

public class MadgwickAHRS {
    public double SamplePeriod;
    public double Quaternion[] = new double[4];
    public double Beta;
    public MadgwickAHRS() {
        this.SamplePeriod = 1.0/256;
        this.Beta = 1;
        this.Quaternion[0] = 1;
        this.Quaternion[1] = 0;
        this.Quaternion[2] = 0;
        this.Quaternion[3] = 0;
    }
    public MadgwickAHRS(double SamplePeriod,double Quaternion[],double Beta){
        this.SamplePeriod = SamplePeriod;
        this.Beta = Beta;
        this.Quaternion[0] = Quaternion[0];
        this.Quaternion[1] = Quaternion[1];
        this.Quaternion[2] = Quaternion[2];
        this.Quaternion[3] = Quaternion[3];
    }
    public void Update(double Gyroscope[], double Accelerometer[],double Magnetometer[]){
        double nA = util.norm(Accelerometer);
        double nM = util.norm(Magnetometer);
        if(nA == 0){
            return;
        }
        else {
            for(int i = 0; i < Accelerometer.length; i ++){
                Accelerometer[i] = Accelerometer[i]/nA;
            }
        }
        if(nM == 0){
            return;
        }
        else {
            for(int i = 0; i < Magnetometer.length; i ++){
                Magnetometer[i] = Magnetometer[i]/nM;
            }
        }
        Matrix q = DenseMatrix.Factory.zeros(1, this.Quaternion.length);
        for( int i = 0 ; i < this.Quaternion.length ; i ++ ){
            q.setAsDouble(this.Quaternion[i],0,i);
        }
        Matrix GyroscopeMatrix = DenseMatrix.Factory.zeros(1, 4);
        GyroscopeMatrix.setAsDouble(0,0,0);
        for( int i = 1 ; i < 4 ; i ++ ){
            GyroscopeMatrix.setAsDouble(Gyroscope[i - 1],0,i);
        }
        //Matrix AccelerometerMatrix = DenseMatrix.Factory.zeros(1, 4);
        Matrix MagnetometerMatrix = DenseMatrix.Factory.zeros(1, 4);
        MagnetometerMatrix.setAsDouble(0,0,0);
        for( int i = 1 ; i < 4 ; i ++ ){
            MagnetometerMatrix.setAsDouble(Magnetometer[i - 1],0,i);
        }
//        System.out.println("q" + q);
//        System.out.println("MagnetometerMatrix" + MagnetometerMatrix);
//        System.out.println("util.quaternConj(q)" + util.quaternConj(q));
//        System.out.println("util.quaternProd(MagnetometerMatrix,util.quaternConj(q))" + util.quaternProd(MagnetometerMatrix,util.quaternConj(q)));
        Matrix h = util.quaternProd(q,util.quaternProd(MagnetometerMatrix,util.quaternConj(q)));
        Matrix b = DenseMatrix.Factory.zeros(1, 4);
        b.setAsDouble(0,0,0);
        double temp[] = new double[2];
        temp[0] = h.getAsDouble(0,1);
        temp[1] = h.getAsDouble(0,2);
        b.setAsDouble(util.norm(temp),0,1);
        b.setAsDouble(0,0,2);
        b.setAsDouble(h.getAsDouble(0,3),0,3);
        Matrix F,J;
        F = DenseMatrix.Factory.zeros(6, 1);
        J = DenseMatrix.Factory.zeros(6, 4);
        F.setAsDouble(2*(q.getAsDouble(0,1)*q.getAsDouble(0,3) - q.getAsDouble(0,0)*q.getAsDouble(0,2)) - Accelerometer[0],0,0);
        F.setAsDouble(2*(q.getAsDouble(0,0)*q.getAsDouble(0,1) + q.getAsDouble(0,2)*q.getAsDouble(0,3)) - Accelerometer[1],1,0);
        F.setAsDouble(2*(0.5 - q.getAsDouble(0,1)*q.getAsDouble(0,1) - q.getAsDouble(0,2)*q.getAsDouble(0,2)) - Accelerometer[2],2,0);
        F.setAsDouble(2*b.getAsDouble(0,1)*(0.5 - q.getAsDouble(0,2)*q.getAsDouble(0,2) - q.getAsDouble(0,3)*q.getAsDouble(0,3)) + 2*b.getAsDouble(0,3)*(q.getAsDouble(0,1)*q.getAsDouble(0,3) - q.getAsDouble(0,0)*q.getAsDouble(0,2)) - Magnetometer[0],3,0);
        F.setAsDouble(2*b.getAsDouble(0,1)*(q.getAsDouble(0,1)*q.getAsDouble(0,2) - q.getAsDouble(0,0)*q.getAsDouble(0,3)) + 2*b.getAsDouble(0,3)*(q.getAsDouble(0,0)*q.getAsDouble(0,1) + q.getAsDouble(0,2)*q.getAsDouble(0,3)) - Magnetometer[1],4,0);
        F.setAsDouble(2*b.getAsDouble(0,1)*(q.getAsDouble(0,0)*q.getAsDouble(0,2) + q.getAsDouble(0,1)*q.getAsDouble(0,3)) + 2*b.getAsDouble(0,3)*(0.5 - q.getAsDouble(0,1)*q.getAsDouble(0,1) - q.getAsDouble(0,2)*q.getAsDouble(0,2)) - Magnetometer[2],5,0);
        J.setAsDouble(-2*q.getAsDouble(0,2),0,0);
        J.setAsDouble(2*q.getAsDouble(0,3),0,1);
        J.setAsDouble(-2*q.getAsDouble(0,0), 0,2);
        J.setAsDouble(2*q.getAsDouble(0,1),0,3);
        J.setAsDouble(2*q.getAsDouble(0,1),1,0);
        J.setAsDouble(2*q.getAsDouble(0,0),1,1);
        J.setAsDouble(2*q.getAsDouble(0,3),1,2);
        J.setAsDouble(2*q.getAsDouble(0,2),1,3);
        J.setAsDouble(0,2,0);
        J.setAsDouble(-4*q.getAsDouble(0,1),2,1);
        J.setAsDouble(-4*q.getAsDouble(0,2),2,2);
        J.setAsDouble(0,2,3);
        J.setAsDouble(-2*b.getAsDouble(0,3)*q.getAsDouble(0,2),3,0);
        J.setAsDouble(2*b.getAsDouble(0,3)*q.getAsDouble(0,3),3,1);
        J.setAsDouble(-4*b.getAsDouble(0,1)*q.getAsDouble(0,2)-2*b.getAsDouble(0,3)*q.getAsDouble(0,0),3,2);
        J.setAsDouble(-4*b.getAsDouble(0,1)*q.getAsDouble(0,3)+2*b.getAsDouble(0,3)*q.getAsDouble(0,1),3,3);
        J.setAsDouble(-2*b.getAsDouble(0,1)*q.getAsDouble(0,3)+2*b.getAsDouble(0,3)*q.getAsDouble(0,1),4,0);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,2)+2*b.getAsDouble(0,3)*q.getAsDouble(0,0),4,1);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,1)+2*b.getAsDouble(0,3)*q.getAsDouble(0,3),4,2);
        J.setAsDouble(-2*b.getAsDouble(0,1)*q.getAsDouble(0,0)+2*b.getAsDouble(0,3)*q.getAsDouble(0,2),4,3);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,2),5,0);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,3)-4*b.getAsDouble(0,3)*q.getAsDouble(0,1),5,1);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,0)-4*b.getAsDouble(0,3)*q.getAsDouble(0,2),5,2);
        J.setAsDouble(2*b.getAsDouble(0,1)*q.getAsDouble(0,1),5,3);
        Matrix step = J.transpose().mtimes(F);
        step = step.divide(util.norm(step));
        Matrix qDot = util.quaternProd(q,GyroscopeMatrix).times(0.5).minus(step.transpose().times(this.Beta));
        q = q.plus(qDot.times(this.SamplePeriod));
        for( int i = 0 ; i < this.Quaternion.length ; i ++ ){
            this.Quaternion[i] = q.getAsDouble(0,i)/util.norm(q);
        }
    }

    public void UpdateIMU(double Gyroscope[], double Accelerometer[]){
        double nA = util.norm(Accelerometer);
        if(nA == 0){
            return;
        }
        else {
            for(int i = 0; i < Accelerometer.length; i ++){
                Accelerometer[i] = Accelerometer[i]/nA;
            }
        }
        Matrix q = DenseMatrix.Factory.zeros(1, this.Quaternion.length);
        for( int i = 0 ; i < this.Quaternion.length ; i ++ ){
            q.setAsDouble(this.Quaternion[i],0,i);
        }
        Matrix GyroscopeMatrix = DenseMatrix.Factory.zeros(1, 4);
        GyroscopeMatrix.setAsDouble(0,0,0);
        for( int i = 1 ; i < 4 ; i ++ ){
            GyroscopeMatrix.setAsDouble(Gyroscope[i - 1],0,i);
        }
        Matrix F,J;
        F = DenseMatrix.Factory.zeros(3, 1);
        J = DenseMatrix.Factory.zeros(3, 4);
        F.setAsDouble(2*(q.getAsDouble(0,1)*q.getAsDouble(0,3) - q.getAsDouble(0,0)*q.getAsDouble(0,2)) - Accelerometer[0],0,0);
        F.setAsDouble(2*(q.getAsDouble(0,0)*q.getAsDouble(0,1) + q.getAsDouble(0,2)*q.getAsDouble(0,3)) - Accelerometer[1],1,0);
        F.setAsDouble(2*(0.5 - q.getAsDouble(0,1)*q.getAsDouble(0,1) - q.getAsDouble(0,2)*q.getAsDouble(0,2)) - Accelerometer[2],2,0);
        J.setAsDouble(-2*q.getAsDouble(0,2),0,0);
        J.setAsDouble(2*q.getAsDouble(0,3),0,1);
        J.setAsDouble(-2*q.getAsDouble(0,0), 0,2);
        J.setAsDouble(2*q.getAsDouble(0,1),0,3);
        J.setAsDouble(2*q.getAsDouble(0,1),1,0);
        J.setAsDouble(2*q.getAsDouble(0,0),1,1);
        J.setAsDouble(2*q.getAsDouble(0,3),1,2);
        J.setAsDouble(2*q.getAsDouble(0,2),1,3);
        J.setAsDouble(0,2,0);
        J.setAsDouble(-4*q.getAsDouble(0,1),2,1);
        J.setAsDouble(-4*q.getAsDouble(0,2),2,2);
        J.setAsDouble(0,2,3);
        Matrix step = J.transpose().mtimes(F);
        step = step.divide(util.norm(step));
        Matrix delta_w = util.quaternProd(q,step.transpose()).times(this.SamplePeriod).times(0.427).times(2);
        Matrix qDot = util.quaternProd(q,GyroscopeMatrix).times(0.5).minus(step.transpose().times(this.Beta));
        q = q.plus(qDot.times(this.SamplePeriod));
        for( int i = 0 ; i < this.Quaternion.length ; i ++ ){
            this.Quaternion[i] = q.getAsDouble(0,i)/util.norm(q);
        }
    }
}
