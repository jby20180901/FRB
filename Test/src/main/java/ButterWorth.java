import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;

public class ButterWorth {
    public double az[];
    public double bz[];
    BUTTER ButterZ[];
    public BUTTORD buttord;
    public ButterWorth(BUTTORD buttord){
        az = new double[buttord.N+1];
        bz = new double[buttord.N+1];
        this.buttord = buttord;
        ButterZ = new BUTTER[buttord.N+1];
        for( int i = 0 ; i < buttord.N + 1 ; i ++){
            ButterZ[i] = new BUTTER(0,0);
        }
        butter(1);
    }

    public void butter( int filter )
    {
        int N = buttord.N;
        double Wn = buttord.Wn;
        BUTTER Butter[] = new BUTTER[N+1];
        complex poles[] = new complex[N];
        complex Res[] = new complex[N + 1];
        complex Res_Save[] = new complex[N + 1];
        double delta = 0.5;
        double a[] = new double[N + 1];
        double b[] = new double[N + 1];
        int count = 0, count_1 = 0;
        for (int k = 0; k < N; k++) {
            double a1 = Math.PI * (delta + (2.0 * k + 1) / (2.0 * N));
            poles[k] = new complex(Wn*Math.cos(a1), Wn*Math.sin(a1)); //
        }
        for(int i = 0; i < N + 1 ; i ++){
            Butter[i] = new BUTTER(0,0);
            Res[i] = new complex(0,0);
            Res_Save[i] = new complex(0,0);
        }
        Res[0] = poles[0];
        Res[1] = new complex(1, 0);
        for (count_1 = 0; count_1 < N - 1; count_1++) {
            for (count = 0; count <= count_1 + 2; count++) {
                if (0 == count) {
                    Res_Save[count] = Res[count].mul(poles[count_1 + 1]);
                }
                else if ((count_1 + 2) == count) {
                    Res_Save[count] = Res_Save[count].add(Res[count - 1]);
                }
                else {
                    Res_Save[count] = Res[count].mul(poles[count_1 + 1]);
                    Res_Save[count] = Res_Save[count].add(Res[count - 1]);
                }
            }
            for (count = 0; count <= N; count++) {
                Res[count] = Res_Save[count];
                a[N - count] = Math.abs(Res[count].getA());
                b[N - count] = 0;
                Butter[N - count].A = Math.abs(Res[count].getA());
                Butter[N - count].B = 0;
            }
            Butter[N].B = Butter[N].A;
		    b[N] = a[N];
        }
        Bilinear(a,b);
        for (count = 0; count <= N; count++) {
            ButterZ[N - count].A = az[N - count];
            ButterZ[N - count].B = bz[N - count];
        }
    }

    private int Bilinear(double as[], double bs[])
    {
        int N = buttord.N;
        int Count = 0, Count_1 = 0, Count_2 = 0, Count_Z = 0;
        double Res[], Res_Save[];
        Res = new double[N + 1];
        Res_Save = new double[N + 1];
        for(int i = 0; i < Res.length ; i ++){
            Res[i] = 0;
        }
        for(int i = 0; i < Res_Save.length ; i ++){
            Res_Save[i] = 0;
        }
        double A = 0;
        for (Count_Z = N; Count_Z >= 0; Count_Z--)
        {
            A += as[Count_Z];
        }
        for (Count_Z = 0; Count_Z <= N; Count_Z++)
        {
            az[Count_Z] = 0;
            bz[Count_Z] = 0;
        }
        for (Count = 0; Count <= N; Count++)
        {
            for (Count_Z = 0; Count_Z <= N; Count_Z++)
            {
                Res[Count_Z] = 0;
                Res_Save[Count_Z] = 0;
            }
            Res_Save[0] = 1;

            for (Count_1 = 0; Count_1 < N - Count; Count_1++)//计算（1-Z^-1）^N-Count的系数,
            {												//Res_Save[]=Z^-1多项式的系数，从常数项开始
                for (Count_2 = 0; Count_2 <= Count_1 + 1; Count_2++)
                {
                    if (Count_2 == 0)
                    {
                        Res[Count_2] += Res_Save[Count_2];
                    }
                    else if ((Count_2 == (Count_1 + 1)) && (Count_1 != 0))
                    {
                        Res[Count_2] += -Res_Save[Count_2 - 1];
                    }
                    else
                    {
                        Res[Count_2] += Res_Save[Count_2] - Res_Save[Count_2 - 1];
                    }
                }
                for (Count_Z = 0; Count_Z <= N; Count_Z++)
                {
                    Res_Save[Count_Z] = Res[Count_Z];
                    Res[Count_Z] = 0;
                }
            }
            for (Count_1 = (N - Count); Count_1 < N; Count_1++)//计算(1-Z^-1)^N-Count*（1+Z^-1）^Count的系数,
            {												//Res_Save[]=Z^-1多项式的系数，从常数项开始
                for (Count_2 = 0; Count_2 <= Count_1 + 1; Count_2++)
                {
                    if (Count_2 == 0)
                    {
                        Res[Count_2] += Res_Save[Count_2];
                    }
                    else if ((Count_2 == (Count_1 + 1)) && (Count_1 != 0))
                    {
                        Res[Count_2] += Res_Save[Count_2 - 1];
                    }
                    else
                    {
                        Res[Count_2] += Res_Save[Count_2] + Res_Save[Count_2 - 1];
                    }
                }
                for (Count_Z = 0; Count_Z <= N; Count_Z++)
                {
                    Res_Save[Count_Z] = Res[Count_Z];
                    Res[Count_Z] = 0;
                }
            }
            for (Count_Z = 0; Count_Z <= N; Count_Z++)
            {
			az[Count_Z] += as[Count] * Res_Save[Count_Z];
			bz[Count_Z] += bs[Count] * Res_Save[Count_Z];
            }
        }//最外层for循环
        for (Count_Z = N; Count_Z >= 0; Count_Z--)
        {
            bz[Count_Z] = bz[Count_Z] / A;//(*(az + 0));
            az[Count_Z] = az[Count_Z] / A;// (*(az + 0));
        }
        return (int)1;
    }

    public Matrix filter( Matrix Angle_V)//z为a，b长度
    {
        int z = buttord.N + 1;
        long n = Angle_V.getRowCount();
        long m = Angle_V.getColumnCount();
        Matrix newAngle_V = DenseMatrix.Factory.zeros(n,m);
        for(int i = 0; i < n; i++){
            for (int k = 0; k < m; k++) {
                long naxpy = m - k;
                if (!(naxpy < z)) {
                    naxpy = z;
                }

                for (int j = 0; j < naxpy; j++) {
                    newAngle_V.setAsDouble( newAngle_V.getAsDouble(i,k + j) + Angle_V.getAsDouble(i,k) * bz[j],i,k + j);
                }

                naxpy = m - 1 - k;
                if (!(naxpy < z-1)) {
                    naxpy = z-1;
                }

                double a1 = -newAngle_V.getAsDouble(i,k);
                for (int j = 1; j <= naxpy; j++) {
                    newAngle_V.setAsDouble(newAngle_V.getAsDouble(i,k + j) + a1 * az[j],i,k + j);
                }
            }
        }
        return newAngle_V;
    }

    public Matrix filter2(Matrix xArr){
        int lenB = buttord.N + 1;
        int lenA = buttord.N + 1;
        long lenX = xArr.getRowCount();
        long m = xArr.getColumnCount();
        int M = lenB - 1;
        int N = lenA - 1;
        Matrix yArr = DenseMatrix.Factory.zeros(lenX,m);
        for(int k = 0; k < m; k ++) {
            for (int i = 0; i < lenX; i++) {
                double yFront = 0;
                for (int j = 0; j <= M && j <= i; j++) {
                    yFront = yFront + bz[j] * xArr.getAsDouble(i - j,k);
                }
                double yBehind = 0;
                for (int j = 1; j <= N && j <= i; j++) {
                    yBehind = yBehind + az[j] * yArr.getAsDouble(i - j,k);
                }
                yArr.setAsDouble((yFront - yBehind) / az[0],i,k) ;
            }
        }
        return yArr;
    }

    public synchronized double[] IIRFilter(double[] signal, double[] a, double[] b) {
        double[] in = new double[b.length];
        double[] out = new double[a.length-1];
        double[] outData = new double[signal.length];
        for (int i = 0; i < signal.length; i++) {

            System.arraycopy(in, 0, in, 1, in.length - 1);
            in[0] = signal[i];

            //calculate y based on a and b coefficients
            //and in and out.
            float y = 0;
            for(int j = 0 ; j < b.length ; j++){
                y += b[j] * in[j];

            }

            for(int j = 0;j < a.length-1;j++){
                y -= a[j+1] * out[j];
            }

            //shift the out array
            System.arraycopy(out, 0, out, 1, out.length - 1);
            out[0] = y;
            outData[i] = y;
        }
        return outData;
    }

    @Override
    public String toString() {
        String ret = "";
        ret += "bz =  [";
        for (int Count_Z = 0; Count_Z <= buttord.N; Count_Z++)
        {
            ret += bz[Count_Z] + " ";
        }
        ret += " ] \n";
        ret += "az =  [";
        for (int Count_Z = 0; Count_Z <= buttord.N; Count_Z++)
        {
            ret += az[Count_Z] + " ";
        }
        ret += " ] \n";
        return ret;
    }
}

class BUTTORD{
    public int N;
    public double  Wn;
    public BUTTORD(int N, double Wn){
        this.N = N;
        this.Wn = Wn;
    }
}

class BUTTER {
    public double  A;
    public double  B;
    public BUTTER(double a,double b){
        A = a;
        B = b;
    }
}