public class complex {
    private double a;
    private double b;
    public double getA() {
        return a;
    }
    public void setA(double a) {
        this.a = a;
    }
    public double getB() {
        return b;
    }
    public void setB(double b) {
        this.b = b;
    }
    public complex() {
        this.a=0.0;
        this.b=0.0;
    }
    public complex(double aa,double bb) {
        this.a=aa;
        this.b=bb;
    }
    //加法
    //(a+bi)+(c+di)=(a+c)+(b+d)i
    public complex add(complex c) {
        complex add=new complex(this.a+c.a,this.b+c.b);
        return add;
    }
    //减法
    //(a+bi)-(c+di)=(a-c)+(b-d)i
    public complex sub(complex c) {
        complex sub=new complex(this.a-c.a,this.b-c.b);
        return sub;
    }
    //乘法
    //(a+bi)*(c+di)=(ac-bd)+(ad+bc)i
    public complex mul(complex c) {
        double aa=this.a*c.a-this.b*c.b;
        double bb=this.a*c.b+this.b*c.a;
        complex mul=new complex(aa,bb);
        return mul;
    }
    //除法
    //(a+bi)/(c+di)=(ac+bd)/(c²+d²)+((bc-ad)/(c²+d²))i
    public complex div(complex c) {
        double x=c.a*c.a+c.b*c.b;
        double aa=(this.a*c.a+this.b*c.b)/x;
        double bb=(this.b*c.a-this.a*c.b)/x;
        complex div=new complex(aa,bb);
        return div;
    }
    //格式
    public String toString() {
        return "("+this.a+","+this.b+")";
    }
}

