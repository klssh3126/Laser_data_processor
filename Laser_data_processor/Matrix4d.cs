using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot_vector
{
    enum MyEnum
    {
        row_type, column_type,
    }

    class Matrix4d
    {
        private double[,] val;

        public Matrix4d()
        {
            val = new double[4, 4];
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    this.val[i, j] = 0.0;
                }
            }
        }


        public Matrix4d(Vector4d vec1, Vector4d vec2, Vector4d vec3, Vector4d vec4, int input_type = (int)MyEnum.row_type)
        {
            val = new double[4, 4];
            if (input_type == 0)
            {
                Row1 = vec1;
                Row2 = vec2;
                Row3 = vec3;
                Row4 = vec4;
            }
            else
            {
                Column1 = vec1;
                Column2 = vec2;
                Column3 = vec3;
                Column4 = vec4;
            }
        }

        public Matrix4d(double[] vec1, double[] vec2, double[] vec3, double[] vec4, int input_type = (int)MyEnum.row_type)
        {
            val = new double[4, 4];
            if (input_type == (int)MyEnum.row_type)
            {
                Row1 = new Vector4d(vec1);
                Row2 = new Vector4d(vec2);
                Row3 = new Vector4d(vec3);
                Row4 = new Vector4d(vec4);
            }
            else
            {
                Column1 = new Vector4d(vec1);
                Column2 = new Vector4d(vec2);
                Column3 = new Vector4d(vec3);
                Column4 = new Vector4d(vec4);
            }
        }

        public double this[int i, int j]
        {
            get { return this.val[i, j]; }
            set { this.val[i, j] = value; }
        }

        public Vector4d Mult(Vector4d a)
        {
            Vector4d v = new Vector4d (0, 0, 0, 0);

            v[0] = this.val[0, 0] * a[0] + this.val[0, 1] * a[1] + this.val[0, 2] * a[2] + this.val[0,3]*a[3];
            v[1] = this.val[1, 0] * a[0] + this.val[1, 1] * a[1] + this.val[1, 2] * a[2] + this.val[1,3]*a[3];
            v[2] = this.val[2, 0] * a[0] + this.val[2, 1] * a[1] + this.val[2, 2] * a[2] + this.val[2,3]*a[3];
            v[3] = this.val[3, 0] * a[0] + this.val[3, 1] * a[1] + this.val[3, 2] * a[2] + this.val[3,3]*a[3];

            return v;
        }

        public Matrix4d Mult(Matrix4d a)
        {
            Matrix4d B = new Matrix4d();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        B[i, j] = B[i, j] + this.val[i, k] * a[k, j];
                    }
                }
            }
            return B;
        }

        public Matrix4d Inverse()
        {
            Matrix4d B = new Matrix4d();
            double det;

            #region calculate matrix element
            B[0, 0] = val[1, 1] * val[2, 2] * val[3, 3] -
             val[1, 1] * val[2, 3] * val[3, 2] -
             val[2, 1] * val[1, 2] * val[3, 3] +
             val[2, 1] * val[1, 3] * val[3, 2] +
             val[3, 1] * val[1, 2] * val[2, 3] -
             val[3, 1] * val[1, 3] * val[2, 2];

            B[1, 0] = -val[1, 0] * val[2, 2] * val[3, 3] +
                      val[1, 0] * val[2, 3] * val[3, 2] +
                      val[2, 0] * val[1, 2] * val[3, 3] -
                      val[2, 0] * val[1, 3] * val[3, 2] -
                      val[3, 0] * val[1, 2] * val[2, 3] +
                      val[3, 0] * val[1, 3] * val[2, 2];

            B[2, 0] = val[1, 0] * val[2, 1] * val[3, 3] -
                     val[1, 0] * val[2, 3] * val[3, 1] -
                     val[2, 0] * val[1, 1] * val[3, 3] +
                     val[2, 0] * val[1, 3] * val[3, 1] +
                     val[3, 0] * val[1, 1] * val[2, 3] -
                     val[3, 0] * val[1, 3] * val[2, 1];

            B[3, 0] = -val[1, 0] * val[2, 1] * val[3, 2] +
                       val[1, 0] * val[2, 2] * val[3, 1] +
                       val[2, 0] * val[1, 1] * val[3, 2] -
                       val[2, 0] * val[1, 2] * val[3, 1] -
                       val[3, 0] * val[1, 1] * val[2, 2] +
                       val[3, 0] * val[1, 2] * val[2, 1];

            B[0, 1] = -val[0, 1] * val[2, 2] * val[3, 3] +
                      val[0, 1] * val[2, 3] * val[3, 2] +
                      val[2, 1] * val[0, 2] * val[3, 3] -
                      val[2, 1] * val[0, 3] * val[3, 2] -
                      val[3, 1] * val[0, 2] * val[2, 3] +
                      val[3, 1] * val[0, 3] * val[2, 2];

            B[1, 1] = val[0, 0] * val[2, 2] * val[3, 3] -
                     val[0, 0] * val[2, 3] * val[3, 2] -
                     val[2, 0] * val[0, 2] * val[3, 3] +
                     val[2, 0] * val[0, 3] * val[3, 2] +
                     val[3, 0] * val[0, 2] * val[2, 3] -
                     val[3, 0] * val[0, 3] * val[2, 2];

            B[2, 1] = -val[0, 0] * val[2, 1] * val[3, 3] +
                      val[0, 0] * val[2, 3] * val[3, 1] +
                      val[2, 0] * val[0, 1] * val[3, 3] -
                      val[2, 0] * val[0, 3] * val[3, 1] -
                      val[3, 0] * val[0, 1] * val[2, 3] +
                      val[3, 0] * val[0, 3] * val[2, 1];

            B[3, 1] = val[0, 0] * val[2, 1] * val[3, 2] -
                      val[0, 0] * val[2, 2] * val[3, 1] -
                      val[2, 0] * val[0, 1] * val[3, 2] +
                      val[2, 0] * val[0, 2] * val[3, 1] +
                      val[3, 0] * val[0, 1] * val[2, 2] -
                      val[3, 0] * val[0, 2] * val[2, 1];

            B[0, 2] = val[0, 1] * val[1, 2] * val[3, 3] -
                     val[0, 1] * val[1, 3] * val[3, 2] -
                     val[1, 1] * val[0, 2] * val[3, 3] +
                     val[1, 1] * val[0, 3] * val[3, 2] +
                     val[3, 1] * val[0, 2] * val[1, 3] -
                     val[3, 1] * val[0, 3] * val[1, 2];

            B[1, 2] = -val[0, 0] * val[1, 2] * val[3, 3] +
                      val[0, 0] * val[1, 3] * val[3, 2] +
                      val[1, 0] * val[0, 2] * val[3, 3] -
                      val[1, 0] * val[0, 3] * val[3, 2] -
                      val[3, 0] * val[0, 2] * val[1, 3] +
                      val[3, 0] * val[0, 3] * val[1, 2];

            B[2, 2] = val[0, 0] * val[1, 1] * val[3, 3] -
                      val[0, 0] * val[1, 3] * val[3, 1] -
                      val[1, 0] * val[0, 1] * val[3, 3] +
                      val[1, 0] * val[0, 3] * val[3, 1] +
                      val[3, 0] * val[0, 1] * val[1, 3] -
                      val[3, 0] * val[0, 3] * val[1, 1];

            B[3, 2] = -val[0, 0] * val[1, 1] * val[3, 2] +
                       val[0, 0] * val[1, 2] * val[3, 1] +
                       val[1, 0] * val[0, 1] * val[3, 2] -
                       val[1, 0] * val[0, 2] * val[3, 1] -
                       val[3, 0] * val[0, 1] * val[1, 2] +
                       val[3, 0] * val[0, 2] * val[1, 1];

            B[0, 3] = -val[0, 1] * val[1, 2] * val[2, 3] +
                      val[0, 1] * val[1, 3] * val[2, 2] +
                      val[1, 1] * val[0, 2] * val[2, 3] -
                      val[1, 1] * val[0, 3] * val[2, 2] -
                      val[2, 1] * val[0, 2] * val[1, 3] +
                      val[2, 1] * val[0, 3] * val[1, 2];

            B[1, 3] = val[0, 0] * val[1, 2] * val[2, 3] -
                     val[0, 0] * val[1, 3] * val[2, 2] -
                     val[1, 0] * val[0, 2] * val[2, 3] +
                     val[1, 0] * val[0, 3] * val[2, 2] +
                     val[2, 0] * val[0, 2] * val[1, 3] -
                     val[2, 0] * val[0, 3] * val[1, 2];

            B[2, 3] = -val[0, 0] * val[1, 1] * val[2, 3] +
                       val[0, 0] * val[1, 3] * val[2, 1] +
                       val[1, 0] * val[0, 1] * val[2, 3] -
                       val[1, 0] * val[0, 3] * val[2, 1] -
                       val[2, 0] * val[0, 1] * val[1, 3] +
                       val[2, 0] * val[0, 3] * val[1, 1];

            B[3, 3] = val[0, 0] * val[1, 1] * val[2, 2] -
                      val[0, 0] * val[1, 2] * val[2, 1] -
                      val[1, 0] * val[0, 1] * val[2, 2] +
                      val[1, 0] * val[0, 2] * val[2, 1] +
                      val[2, 0] * val[0, 1] * val[1, 2] -
                      val[2, 0] * val[0, 2] * val[1, 1];
#endregion
            det = val[0, 0] * B[0, 0] + val[0, 1] * B[1, 0] + val[0, 2] * B[2, 0] + val[0, 3] * B[3, 0];

            //Console.WriteLine("Determinant : {0}", det);
            if (det == 0)
            {
                throw new Exception("Matrix is singular");
            }


            double det_ = 1.0 / det;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    B[i, j] = B[i, j] * det_;
                }
            }

            return B;
        }

        #region Row, Column Property
        public Vector4d Row1
        {
            get { return new Vector4d(this.val[0, 0], this.val[0, 1], this.val[0, 2], this.val[0, 3]); }
            set
            {
                this.val[0, 0] = value.X;
                this.val[0, 1] = value.Y;
                this.val[0, 2] = value.Z;
                this.val[0, 3] = value.W;
            }
        }
        public Vector4d Row2
        {
            get { return new Vector4d(this.val[1, 0], this.val[1, 1], this.val[1, 2], this.val[1, 3]); }
            set
            {
                this.val[1, 0] = value.X;
                this.val[1, 1] = value.Y;
                this.val[1, 2] = value.Z;
                this.val[1, 3] = value.W;
            }
        }
        public Vector4d Row3
        {
            get { return new Vector4d(this.val[2, 0], this.val[2, 1], this.val[2, 2], this.val[2, 3]); }
            set
            {
                this.val[2, 0] = value.X;
                this.val[2, 1] = value.Y;
                this.val[2, 2] = value.Z;
                this.val[2, 3] = value.W;
            }
        }
        public Vector4d Row4
        {
            get { return new Vector4d(this.val[3, 0], this.val[3, 1], this.val[3, 2], this.val[3, 3]); }
            set
            {
                this.val[3, 0] = value.X;
                this.val[3, 1] = value.Y;
                this.val[3, 2] = value.Z;
                this.val[3, 3] = value.W;
            }
        }
        public Vector4d Column1
        {
            get { return new Vector4d(this.val[0, 0], this.val[1, 0], this.val[2, 0], this.val[3, 0]); }
            set
            {
                this.val[0, 0] = value.X;
                this.val[1, 0] = value.Y;
                this.val[2, 0] = value.Z;
                this.val[3, 0] = value.W;
            }
        }
        public Vector4d Column2
        {
            get { return new Vector4d(this.val[0, 1], this.val[1, 1], this.val[2, 1], this.val[3, 1]); }
            set
            {
                this.val[0, 1] = value.X;
                this.val[1, 1] = value.Y;
                this.val[2, 1] = value.Z;
                this.val[3, 1] = value.W;
            }
        }

        public Vector4d Column3
        {
            get { return new Vector4d(this.val[0, 2], this.val[1, 2], this.val[2, 2], this.val[3, 2]); }
            set
            {
                this.val[0, 2] = value.X;
                this.val[1, 2] = value.Y;
                this.val[2, 2] = value.Z;
                this.val[3, 2] = value.W;
            }
        }

        public Vector4d Column4
        {
            get { return new Vector4d(this.val[0, 3], this.val[1, 3], this.val[2, 3], this.val[3, 3]); }
            set
            {
                this.val[0, 3] = value.X;
                this.val[1, 3] = value.Y;
                this.val[2, 3] = value.Z;
                this.val[3, 3] = value.W;
            }
        }
        #endregion

        public static Matrix4d operator *(Matrix4d m, Matrix4d a)
        {
            return m.Mult(a);
        }
        public override string ToString()
        {
            string str = string.Format("Row1 -> ({0,10:g5}, {1,10:g5}, {2,10:g5}, {3,10:g5})", this.val[0, 0], this.val[0, 1], this.val[0, 2], this.val[0, 3]) + System.Environment.NewLine;
            str += string.Format("Row2 -> ({0,10:g5}, {1,10:g5}, {2,10:g5}, {3,10:g5})", this.val[1, 0], this.val[1, 1], this.val[1, 2], this.val[1, 3]) + System.Environment.NewLine;
            str += string.Format("Row3 -> ({0,10:g5}, {1,10:g5}, {2,10:g5}, {3,10:g5})", this.val[2, 0], this.val[2, 1], this.val[2, 2], this.val[2, 3]) + System.Environment.NewLine;
            str += string.Format("Row4 -> ({0,10:g5}, {1,10:g5}, {2,10:g5}, {3,10:g5})", this.val[3, 0], this.val[3, 1], this.val[3, 2], this.val[3, 3]) + System.Environment.NewLine;
            return str;
        }

    }
}

