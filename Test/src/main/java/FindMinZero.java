import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;
import org.ujmp.core.calculation.Calculation;
import org.ujmp.core.doublematrix.calculation.general.statistical.Min;

import java.util.ArrayList;
import java.util.List;

public class FindMinZero {
    public Matrix y = DenseMatrix.Factory.zeros( 2,1);
    public int slot;
    public Matrix MinZero = DenseMatrix.Factory.zeros( 4,1);
    public double before,after;
    public double beforex,afterx;
    public int tend;
    public double MIN1,ZERO1,MAX1,ZERO2,MIN2,ZERO3,MAX2,ZERO4;//11 21 12 22
    public double MIN1x,ZERO1x,MAX1x,ZERO2x,MIN2x,ZERO3x,MAX2x,ZERO4x;//11 21 12 22
    public int position;
    public FindMinZero(Matrix a){
        System.out.println(a);
        this.y = this.y.appendHorizontally(Calculation.Ret.NEW,a);
        System.out.println(this.y);
        this.slot = 1;
        this.tend = 0;
        this.position = 11;
        this.find();
    }

    private void find(){
        while(this.slot < y.getColumnCount()-1){
            this.Next();
        }
    }

    private void Next(){
        this.before = this.y.getAsDouble(0,this.slot);
        this.after = this.y.getAsDouble(0,this.slot + 1);
        this.beforex = this.y.getAsDouble(1,this.slot);
        this.afterx = this.y.getAsDouble(1,this.slot + 1);
        if(this.tend == 0){
            this.tend = ( this.before > this.after ) ? -1 : 1;
        }
        else {
            this.judge();
        }
        this.slot ++;
    }

    public void judge(){
        int newTend = ( this.before > this.after ) ? -1 : 1;
        if(this.tend == 1){
            if(newTend == -1){
                if(this.position == 21){
                    System.out.println(12);
                    this.position = 12;
                    this.MAX1 = this.before;
                    this.MAX1x = this.beforex;
                }
                else if(this.position == 22){
                    if(this.MAX1 < this.before){
                        this.MAX1 = this.before;
                        this.MAX1x = this.beforex;
                        this.position = 12;
                        System.out.println(12);
                    }
                    else{
                        this.position = 11;
                        this.MAX2 = this.before;
                        this.MAX2x = this.beforex;
                        System.out.println(11);
                        System.out.println("lllllllllll");
                        this.putin();
                    }
                }
            }
        }
        else if(this.tend == -1){
            if(newTend == 1){
                if(this.position == 11){
                    this.position = 21;
                    System.out.println(21);
                    this.MIN1 = this.before;
                    this.MIN1x = this.beforex;
                }
                else if(this.position == 12){
                    if(this.MIN1 > this.before){
                        this.MIN1 = this.before;
                        this.MIN1x = this.beforex;
                        this.position = 21;
                        System.out.println(21);
                    }
                    else{
                        this.position = 22;
                        System.out.println(22);
                        this.MIN2 = this.before;
                        this.MIN2x = this.beforex;
                    }
                }
            }
        }
        if(this.before*this.after < 0){
            if(this.position == 11){
                this.ZERO4 = (Math.abs(this.before) > Math.abs(this.after))?this.after:this.before;
                this.ZERO4x = (Math.abs(this.before) > Math.abs(this.after))?this.afterx:this.beforex;
            }
            else if(this.position == 21){
                this.ZERO1 = (Math.abs(this.before) > Math.abs(this.after))?this.after:this.before;
                this.ZERO1x = (Math.abs(this.before) > Math.abs(this.after))?this.afterx:this.beforex;
            }
            else if(this.position == 12){
                this.ZERO2 = (Math.abs(this.before) > Math.abs(this.after))?this.after:this.before;
                this.ZERO2x = (Math.abs(this.before) > Math.abs(this.after))?this.afterx:this.beforex;
            }
            else if(this.position == 22){
                this.ZERO3 = (Math.abs(this.before) > Math.abs(this.after))?this.after:this.before;
                this.ZERO3x = (Math.abs(this.before) > Math.abs(this.after))?this.afterx:this.beforex;
            }
        }
        this.tend = newTend;
    }

    public void putin(){
        Matrix temp = DenseMatrix.Factory.zeros( 4,1);
        temp.setAsDouble(this.MIN1,0,0);
        temp.setAsDouble(this.MIN1x,1,0);
        temp.setAsDouble(this.ZERO2,2,0);
        temp.setAsDouble(this.ZERO2x,3,0);
        MinZero = MinZero.appendHorizontally(Calculation.Ret.NEW,temp);
    }

    public void getNew(Matrix a,Matrix b){
        this.y.appendHorizontally(Calculation.Ret.NEW,a);
        this.Next();
        this.find();
    }
}
