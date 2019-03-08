using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Math;

namespace robot_vector
{
    class Vector3d
    {
        private double[] val;
        public Vector3d()
        {
            this.val = new double[3];
            this.val[0] = 0.0;
            this.val[1] = 0.0;
            this.val[2] = 0.0;
        }
        public Vector3d(double X, double Y, double Z)
        {
            this.val = new double[3];
            this.val[0] = X;
            this.val[1] = Y;
            this.val[2] = Z;
        }
        public Vector3d(double[] a)
        {
            this.val = new double[3];
            this.val[0] = a[0];
            this.val[1] = a[1];
            this.val[2] = a[2];
        }
        public Vector3d(Point3d p)
        {
            this.val = new double[3];
            this.val[0] = p.X;
            this.val[1] = p.Y;
            this.val[2] = p.Z;
        }
        public Vector3d(Point3d p1, Point3d p2)
        {
            this.val = new double[3];
            this.val[0] = p2.X - p1.X;
            this.val[1] = p2.Y - p1.Y;
            this.val[2] = p2.Z - p1.Z;
        }
        public double X
        {
            get { return val[0]; }
            set { val[0] = value; }
        }
        public double Y
        {
            get { return val[1]; }
            set { val[1] = value; }
        }
        public double Z
        {
            get { return val[2]; }
            set { val[2] = value; }
        }
        public double Norm
        {
            get { return Sqrt(Math.Pow(val[0], 2) + Math.Pow(val[1], 2) + Math.Pow(val[2], 2)); }
        }

        public Vector3d Normalized
        {
            get
            {
                Vector3d tmp = this.Copy();
                double tmp_norm = this.Norm;
                tmp[0] = val[0] / tmp_norm;
                tmp[1] = val[1] / tmp_norm;
                tmp[2] = val[2] / tmp_norm;
                return tmp;
            }
        }
        public double Dot(Vector3d v) //벡터의 내적
        {
            return this.val[0] * v.val[0] + this.val[1] * v.val[1] + this.val[2] * v.val[2];
        }

        public Vector3d Cross(Vector3d v)
        {
            double x = this.Y * v.Z - this.Z * v.Y;
            double y = this.Z * v.X - this.X * v.Z;
            double z = this.X * v.Y - this.Y * v.X;
            return new Vector3d(x, y, z);
        }

        public Vector3d Copy()
        {
            return new Vector3d(val);
        }

        public double this[int i]
        {
            get { return val[i]; }
            set { val[i] = value; }
        }

        public override String ToString()
        {
            Vector3d r = this;
            return string.Format(" Vector -> (E:{0,10:g5}, N:{1,10:g5}, U:{2,10:g5})", r.X, r.Y, r.Z) + System.Environment.NewLine;
        }

        public Vector3d Mult(double a)
        {
            Vector3d tmp = this.Copy();
            tmp[0] *= a;
            tmp[1] *= a;
            tmp[2] *= a;
            return tmp;
        }
        public Matrix3d Make_Skew_Matrix()
        {

            double[] row1 = { 0, -Z, Y };
            double[] row2 = { Z, 0, -X };
            double[] row3 = { -Y, X, 0};

            return new Matrix3d(row1,row2,row3);
        }

        public static Vector3d operator -(Vector3d v)
        {
            return v.Mult(-1.0);
        }
    }
}
