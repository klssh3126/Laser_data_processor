using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot_vector
{
    class Vector4d
    {
        private double[] val;

        public Vector4d()
        {
            this.val = new double[4];
            this.val[0] = 0.0;
            this.val[1] = 0.0;
            this.val[2] = 0.0;
            this.val[3] = 0.0;
        }

        public Vector4d(double X, double Y, double Z, double W)
        {
            this.val = new double[4];
            this.val[0] = X;
            this.val[1] = Y;
            this.val[2] = Z;
            this.val[3] = W;
        }

        public Vector4d(double[] a)
        {
            this.val = new double[4];
            this.val[0] = a[0];
            this.val[1] = a[1];
            this.val[2] = a[2];
            this.val[3] = a[3];
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
        public double W
        {
            get { return val[3]; }
            set { val[3] = value; }
        }

        public double this[int i]
        {
            get { return val[i]; }
            set { val[i] = value; }
        }

        public override String ToString()
        {
            Vector4d r = this;
            string str = string.Format(" Vector -> ({0,10:g5}, {1,10:g5}, {2,10:g5}) , {3,10:g5}", r.X, r.Y, r.Z, r.W) + System.Environment.NewLine;
            return str;
        }
    }
}
