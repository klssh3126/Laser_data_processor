using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot_vector
{
    class Matrix3d
    {
        private double[,] val;

        public Matrix3d()
        {
            val = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    this.val[i, j] = 0.0;
                }
            }
        }
        public Matrix3d(Vector3d row1, Vector3d row2, Vector3d row3)
        {
            val = new double[3, 3];
            Row1 = row1;
            Row2 = row2;
            Row3 = row3;
        }
        public Matrix3d(double[] row1, double[] row2, double[] row3)
        {
            val = new double[3, 3];
            Row1 = new Vector3d(row1);
            Row2 = new Vector3d(row2);
            Row3 = new Vector3d(row3);
        }

        public double this[int i, int j]
        {
            get { return this.val[i, j]; }
            set { this.val[i, j] = value; }
        }

        public Vector3d Row1
        {
            get { return new Vector3d(this.val[0, 0], this.val[0, 1], this.val[0, 2]); }
            set
            {
                this.val[0, 0] = value.X;
                this.val[0, 1] = value.Y;
                this.val[0, 2] = value.Z;
            }
        }
        public Vector3d Row2
        {
            get { return new Vector3d(this.val[1, 0], this.val[1, 1], this.val[1, 2]); }
            set
            {
                this.val[1, 0] = value.X;
                this.val[1, 1] = value.Y;
                this.val[1, 2] = value.Z;
            }
        }
        public Vector3d Row3
        {
            get { return new Vector3d(this.val[2, 0], this.val[2, 1], this.val[2, 2]); }
            set
            {
                this.val[2, 0] = value.X;
                this.val[2, 1] = value.Y;
                this.val[2, 2] = value.Z;
            }
        }
        public double Det
        {
            get
            {
                double k11 = this.val[2, 2] * this.val[1, 1] - this.val[2, 1] * this.val[1, 2];
                double k12 = this.val[2, 1] * this.val[0, 2] - this.val[2, 2] * this.val[0, 1];
                double k13 = this.val[1, 2] * this.val[0, 1] - this.val[1, 1] * this.val[0, 2];

                return this.val[0, 0] * k11 + this.val[1, 0] * k12 + this.val[2, 0] * k13;
            }
        }
        public Matrix3d Add(Matrix3d a)
        {
            Matrix3d B = new Matrix3d();
            for (int i = 0; i <= 2; i++)
            {
                for (int j = 0; j <= 2; j++)
                {
                    B[i, j] = this.val[i, j] + a[i, j];
                }
            }
            return B;
        }
        public Matrix3d Subtract(Matrix3d a)
        {
            Matrix3d B = new Matrix3d();
            for (int i = 0; i <= 2; i++)
            {
                for (int j = 0; j <= 2; j++)
                {
                    B[i, j] = this.val[i, j] - a[i, j];
                }
            }
            return B;
        }

        public Matrix3d Mult(double a)
        {
            Matrix3d B = new Matrix3d();
            for (int i = 0; i <= 2; i++)
            {
                for (int j = 0; j <= 2; j++)
                {
                    B[i, j] = a * this.val[i, j];
                }
            }
            return B;
        }

        public Vector3d Mult(Vector3d a)
        {
            Vector3d b = new Vector3d(0, 0, 0);
            b[0] = this.val[0, 0] * a[0] + this.val[0, 1] * a[1] + this.val[0, 2] * a[2];
            b[1] = this.val[1, 0] * a[0] + this.val[1, 1] * a[1] + this.val[1, 2] * a[2];
            b[2] = this.val[2, 0] * a[0] + this.val[2, 1] * a[1] + this.val[2, 2] * a[2];
            return b;
        }
        public Point3d Mult(Point3d p)
        {
            double x = this.val[0, 0] * p.X + this.val[0, 1] * p.Y + this.val[0, 2] * p.Z;
            double y = this.val[1, 0] * p.X + this.val[1, 1] * p.Y + this.val[1, 2] * p.Z;
            double z = this.val[2, 0] * p.X + this.val[2, 1] * p.Y + this.val[2, 2] * p.Z;
            return new Point3d(x, y, z);
        }

        public Matrix3d Mult(Matrix3d a)
        {
            Matrix3d B = new Matrix3d();
            for (int i = 0; i <= 2; i++)
            {
                for (int j = 0; j <= 2; j++)
                {
                    for (int k = 0; k <= 2; k++)
                    {
                        B[i, j] = B[i, j] + this.val[i, k] * a[k, j];
                    }
                }
            }
            return B;
        }

        public static Matrix3d Identity()
        {
            Matrix3d I = new Matrix3d();
            I[0, 0] = 1.0;
            I[1, 1] = 1.0;
            I[2, 2] = 1.0;
            return I;
        }

        public Matrix3d Inverse()
        {
            Matrix3d B = new Matrix3d();

            double k11 = this.val[2, 2] * this.val[1, 1] - this.val[2, 1] * this.val[1, 2];
            double k12 = this.val[2, 1] * this.val[0, 2] - this.val[2, 2] * this.val[0, 1];
            double k13 = this.val[1, 2] * this.val[0, 1] - this.val[1, 1] * this.val[0, 2];
            double k21 = this.val[2, 0] * this.val[1, 2] - this.val[2, 2] * this.val[1, 0];
            double k22 = this.val[2, 2] * this.val[0, 0] - this.val[2, 0] * this.val[0, 2];
            double k23 = this.val[1, 0] * this.val[0, 2] - this.val[1, 2] * this.val[0, 0];
            double k31 = this.val[2, 1] * this.val[1, 0] - this.val[2, 0] * this.val[1, 1];
            double k32 = this.val[2, 0] * this.val[0, 1] - this.val[2, 1] * this.val[0, 0];
            double k33 = this.val[1, 1] * this.val[0, 0] - this.val[1, 0] * this.val[0, 1];

            double det = this.val[0, 0] * k11 + this.val[1, 0] * k12 + this.val[2, 0] * k13;

            if (det != 0.0)
            {
                B[0, 0] = k11 / det;
                B[0, 1] = k12 / det;
                B[0, 2] = k13 / det;

                B[1, 0] = k21 / det;
                B[1, 1] = k22 / det;
                B[1, 2] = k23 / det;

                B[2, 0] = k31 / det;
                B[2, 1] = k32 / det;
                B[2, 2] = k33 / det;
            }
            else
            {
                throw new Exception("Matrix is singular");
            }

            return B;
        }

        public override string ToString()
        {
            string str = string.Format("Row1 -> ({0,10:g5}, {1,10:g5}, {2,10:g5})", this.val[0, 0], this.val[0, 1], this.val[0, 2]) + System.Environment.NewLine;
            str += string.Format("Row2 -> ({0,10:g5}, {1,10:g5}, {2,10:g5})", this.val[1, 0], this.val[1, 1], this.val[1, 2]) + System.Environment.NewLine;
            str += string.Format("Row3 -> ({0,10:g5}, {1,10:g5}, {2,10:g5})", this.val[2, 0], this.val[2, 1], this.val[2, 2]);
            return str;
        }
        public static Matrix3d operator +(Matrix3d m, Matrix3d a)
        {
            return m.Add(a);
        }
        // "-"
        public static Matrix3d operator -(Matrix3d m)
        {
            return m.Mult(-1.0);
        }
        public static Matrix3d operator -(Matrix3d m, Matrix3d a)
        {
            return m.Subtract(a);
        }
        public static Matrix3d operator *(Matrix3d m, double a)
        {
            return m.Mult(a);
        }
        public static Matrix3d operator *(double a, Matrix3d m)
        {
            return m.Mult(a);
        }
        public static Vector3d operator *(Matrix3d m, Vector3d a)
        {
            return m.Mult(a);
        }
        public static Point3d operator *(Matrix3d m, Point3d p)
        {
            return m.Mult(p);
        }
        public static Matrix3d operator *(Matrix3d m, Matrix3d a)
        {
            return m.Mult(a);
        }
    }
}
