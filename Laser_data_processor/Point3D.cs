using System;
using static System.Math;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot_vector
{
    class Point3d
    {
        private double _x;
        private double _y;
        private double _z;

        public Point3d()
        {
            _x = 0.0;
            _y = 0.0;
            _z = 0.0;
        }
        public Point3d(double x, double y, double z)
        {
            _x = x;
            _y = y;
            _z = z;
        }
        public Point3d(double[] a)
        {
            _x = a[0];
            _y = a[1];
            _z = a[2];
        }
        public double X
        {
            get { return _x; }
            set { _x = value; }
        }
        public double Y
        {
            get { return _y; }
            set { _y = value; }
        }
        public double Z
        {
            get { return _z; }
            set { _z = value; }
        }
        public Point3d Copy()
        {
            return new Point3d(_x, _y, _z);
        }

        public Point3d Add(Point3d p)
        {
            Point3d tmp = this.Copy();
            tmp.X += p.X;
            tmp.Y += p.Y;
            tmp.Z += p.Z;
            return tmp;
        }
        public Point3d Subtract(Point3d p)
        {
            Point3d tmp = this.Copy();
            tmp.X -= p.X;
            tmp.Y -= p.Y;
            tmp.Z -= p.Z;
            return tmp;
        }
        public Point3d Scale(double a)
        {
            Point3d tmp = this.Copy();
            tmp.X *= a;
            tmp.Y *= a;
            tmp.Z *= a;
            return tmp;
        }
        
        public static Point3d operator +(Point3d v, Point3d a)
        {
            return v.Add(a);
        }
        public static Point3d operator -(Point3d v, Point3d a)
        {
            return v.Subtract(a);
        }
        public static Point3d operator -(Point3d v)
        {
            return v.Scale(-1.0);
        }
        public  static Point3d operator /(Point3d v, double a)
        {
            return v.Scale(1.0 / a);
        }
        public override String ToString()
        {

            Point3d p = this;
            string str = string.Format("Point3d -> ({0,10:0.##}, {1,10:0.##}, {2,10:0.##})", p.X, p.Y, p.Z) + System.Environment.NewLine;
            return str;
        }
        public bool BelongsTo(Plane3d s)
        {
            return Abs(s.A * X + s.B * Y + s.C * Z + s.D) == 0; //점이 평면에 속해있는지 아닌지 판단.
        }

        public double DistanceTo(Line3d l)
        {
            Vector3d v = new Vector3d(this, l.Point);
            return v.Cross(l.Direction).Norm / l.Direction.Norm;
        }
    }
}
