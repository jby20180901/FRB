import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;

public class util {
    private int id = 1;
    public util(){

    }
    public static double norm(double a[]){
        double sum = 0;
        for(int i = 0; i < a.length; i ++){
            sum += a[i] * a[i];
        }
        sum = Math.sqrt(sum);
        return sum;
    }
    public static double norm(Matrix a){
        double sum = 0;
        for(int i = 0; i < a.getColumnCount(); i ++){
            sum += a.getAsDouble(0,i) * a.getAsDouble(0,i);
        }
        sum = Math.sqrt(sum);
        return sum;
    }
    public static Matrix quaternProd(Matrix a, Matrix b){
        Matrix ab = Matrix.Factory.zeros(a.getRowCount(), a.getColumnCount());
        Matrix ab1 = ab.selectColumns(ab.NEW, 0);
//        System.out.println("a" + a);
//        System.out.println("b" + b);
        ab1 = a.selectColumns(a.NEW, 0).times(b.selectColumns(b.NEW, 0))
            .minus(a.selectColumns(a.NEW, 1).times(b.selectColumns(b.NEW, 1)))
            .minus(a.selectColumns(a.NEW, 2).times(b.selectColumns(b.NEW, 2)))
            .minus(a.selectColumns(a.NEW, 3).times(b.selectColumns(b.NEW, 3)));
//        System.out.println("ab1" + ab1);
        for (int i = 0 ; i < a.getRowCount(); ++i){
            ab.setAsDouble(ab1.getAsDouble(i,0), i,0);
        }
//        System.out.println("ab" + ab);
        Matrix ab2 = ab.selectColumns(ab.NEW, 1);
        ab2 = a.selectColumns(a.NEW, 0).times(b.selectColumns(b.NEW, 1))
                .plus(a.selectColumns(a.NEW, 1).times(b.selectColumns(b.NEW, 0)))
                .plus(a.selectColumns(a.NEW, 2).times(b.selectColumns(b.NEW, 3)))
                .minus(a.selectColumns(a.NEW, 3).times(b.selectColumns(b.NEW, 2)));
//        System.out.println("ab2" + ab2);
        for (int i = 0 ; i < a.getRowCount(); ++i){
            ab.setAsDouble(ab2.getAsDouble(i,0), i,1);
        }
//        System.out.println("ab" + ab);
        Matrix ab3 = ab.selectColumns(ab.NEW, 2);
        ab3 = a.selectColumns(a.NEW, 0).times(b.selectColumns(b.NEW, 2))
                .minus(a.selectColumns(a.NEW, 1).times(b.selectColumns(b.NEW, 3)))
                .plus(a.selectColumns(a.NEW, 2).times(b.selectColumns(b.NEW, 0)))
                .plus(a.selectColumns(a.NEW, 3).times(b.selectColumns(b.NEW, 1)));
//        System.out.println("ab3" + ab3);
        for (int i = 0 ; i < a.getRowCount(); ++i){
            ab.setAsDouble(ab3.getAsDouble(i,0), i,2);
        }
//        System.out.println("ab" + ab);
        Matrix ab4 = ab.selectColumns(ab.NEW, 3);
        ab4 = a.selectColumns(a.NEW, 0).times(b.selectColumns(b.NEW, 3))
                .plus(a.selectColumns(a.NEW, 1).times(b.selectColumns(b.NEW, 2)))
                .minus(a.selectColumns(a.NEW, 2).times(b.selectColumns(b.NEW, 1)))
                .plus(a.selectColumns(a.NEW, 3).times(b.selectColumns(b.NEW, 0)));
//        System.out.println("ab4" + ab4);
        for (int i = 0 ; i < a.getRowCount(); ++i){
            ab.setAsDouble(ab4.getAsDouble(i,0), i,3);
        }
//        System.out.println("ab" + ab);
        return ab;
    }
    public static Matrix quaternConj(Matrix q){
        Matrix qConj = DenseMatrix.Factory.zeros(q.getRowCount(), 4);
        for (int i = 0 ; i < q.getRowCount(); ++i){
            //可以使用setXXX来进行矩阵的赋值，其中第一个参数是值，第二个参数是行，第三个参数是列
            qConj.setAsDouble(q.getAsDouble(i,0), i,0);
            qConj.setAsDouble(-q.getAsDouble(i,1), i,1);
            qConj.setAsDouble(-q.getAsDouble(i,2), i,2);
            qConj.setAsDouble(-q.getAsDouble(i,3), i,3);
        }
        return qConj;
    }

    public static Matrix quatern2euler(Matrix q){
        Matrix R11,R21,R31,R32,R33;
        Matrix phi,theta,psi;
        R11 = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        R21 = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        R31 = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        R32 = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        R33 = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        phi = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        theta = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        psi = DenseMatrix.Factory.zeros(q.getRowCount(), 1);
        R11 = q.selectColumns(q.NEW, 0)
                .times(q.selectColumns(q.NEW, 0))
                .times(2)
                .minus(1)
                .plus(q.selectColumns(q.NEW, 1)
                        .times(q.selectColumns(q.NEW, 1))
                        .times(2));
        R21 = q.selectColumns(q.NEW, 1)
                .times(q.selectColumns(q.NEW, 2))
                .minus(q.selectColumns(q.NEW, 0)
                        .times(q.selectColumns(q.NEW, 3)))
                .times(2);
        R31 = q.selectColumns(q.NEW, 1)
                .times(q.selectColumns(q.NEW, 3))
                .minus(q.selectColumns(q.NEW, 0)
                        .times(q.selectColumns(q.NEW, 2)))
                .times(2);
        R32 = q.selectColumns(q.NEW, 2)
                .times(q.selectColumns(q.NEW, 3))
                .minus(q.selectColumns(q.NEW, 0)
                        .times(q.selectColumns(q.NEW, 1)))
                .times(2);
        R33 = q.selectColumns(q.NEW, 0)
                .times(q.selectColumns(q.NEW, 0))
                .times(2)
                .minus(1)
                .plus(q.selectColumns(q.NEW, 3)
                        .times(q.selectColumns(q.NEW, 3))
                        .times(2));
        for( int i = 0; i < q.getRowCount() ; i ++ ){
            phi.setAsDouble(
                    Math.atan2(
                            R32.getAsDouble(i,0)
                            ,R33.getAsDouble(i,0))
                    ,i,0);
            theta.setAsDouble(
                    -Math.atan(
                            R31.getAsDouble(i,0)
                                    /Math.sqrt(1-
                                    R31.getAsDouble(i,0) *R31.getAsDouble(i,0)))
                    ,i,0);
            psi.setAsDouble(
                    Math.atan2(
                            R21.getAsDouble(i,0)
                            ,R11.getAsDouble(i,0))
                    ,i,0);
        }
        Matrix euler = DenseMatrix.Factory.zeros(q.getRowCount(), 3);
        for(int i = 0; i < q.getRowCount(); i ++){
            euler.setAsDouble(phi.getAsDouble(i,0),i,0);
            euler.setAsDouble(theta.getAsDouble(i,0),i,1);
            euler.setAsDouble(psi.getAsDouble(i,0),i,2);
        }
        return euler;
    }
}
