using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Math;
using Laser_data_processor;

namespace robot_vector
{
    class Plane3d
    {
        private Point3d _point;
        private Vector3d _normal;

        public Plane3d()
        {
            _point = new Point3d(0, 0, 0);
            _normal = new Vector3d(0, 0, 1);
        }

        public Plane3d(double a, double b, double c, double d)
        {

            if(Abs(a) > Abs(b) && Abs(a) > Abs(c))
            {
                _point = new Point3d(-d / a, 0, 0);
            }
            else if(Abs(b) > Abs(a) && Abs(b) > Abs(c))
            {
                _point = new Point3d(0, -d / b, 0);
            }
            else
            {
                _point = new Point3d(0, 0, -d / c);
            }
            _normal = new Vector3d(a, b, c);
        }

        public Plane3d(Point3d p1, Point3d p2, Point3d p3)
        {
            Vector3d v1 = new Vector3d(p1, p2);
            Vector3d v2 = new Vector3d(p1, p3);
            _normal = v1.Cross(v2);
            _point = p1.Copy();
        }

        public Plane3d(Point3d p1, Vector3d v1)
        {
            _normal = v1.Copy();
            _point = p1.Copy();
        }

        public Point3d Point
        {
            get { return _point.Copy(); }
            set { _point = value.Copy(); }
        }
        
        public Vector3d Normal
        {
            get { return _normal.Copy(); }
            set { _normal = value.Copy(); }
        }

        /// <summary>
        /// Get intersection of two planes.
        /// Returns 'null' (no intersection) or object of type 'Line3d' or 'Plane3d'.
        /// </summary>
        public object IntersectionWith(Plane3d s2)
        {
            Vector3d v = this.Normal.Cross(s2.Normal);
            if (v.Norm  ==0)
            {
                // Planes are coplanar
                if (this.Point.BelongsTo(s2))
                {
                    return this;
                }
                else
                {
                    return null;
                }

            }
            else
            {
                // Find the common point for two planes by intersecting with third plane
                // (using the 'most orthogonal' plane)
                // This part needs to be rewritten
                if (Abs(v.X) >= Abs(v.Y) && Abs(v.X) >= Abs(v.Z))
                {
                    Point3d p = (Point3d)this.IntersectionWith(s2, new Plane3d(1, 0, 0, 0));
                    return new Line3d(p, v); //평면의 교선이다. 우리 프로젝트에서 주의할점은 v의 방향이다.!! UP방향!
                }
                else if (Abs(v.Y) >= Abs(v.X) && Abs(v.Y) >= Abs(v.Z))
                {
                    Point3d p = (Point3d)this.IntersectionWith(s2, new Plane3d(0, 1 ,0 ,0));
                    return new Line3d(p, v);
                }
                else
                {
                    Point3d p = (Point3d)this.IntersectionWith(s2, new Plane3d(0, 0, 1, 0));
                    return new Line3d(p, v);
                }
            }
        }

        public object IntersectionWith(Plane3d s2, Plane3d s3)
        {
            // Set all planes to global CS
            double det = new Matrix3d(new[] { A, B, C }, new[] { s2.A, s2.B, s2.C }, new[] { s3.A, s3.B, s3.C }).Det;
            //각 평면의 법선벡터 X,Y,Z를 행렬로 입력

            if (Abs(det) == 0)
            {
                // Two planes are parallel, third plane is not
                Console.WriteLine("세 평면의 교차결과 평면 또는 교선이 반환되었습니다.\n");
                return null;
            }
            else
            {
                double x = -new Matrix3d(new[] { D, B, C }, new[] { s2.D, s2.B, s2.C }, new[] { s3.D, s3.B, s3.C }).Det / det;
                double y = -new Matrix3d(new[] { A, D, C }, new[] { s2.A, s2.D, s2.C }, new[] { s3.A, s3.D, s3.C }).Det / det;
                double z = -new Matrix3d(new[] { A, B, D }, new[] { s2.A, s2.B, s2.D }, new[] { s3.A, s3.B, s3.D }).Det / det;
                return new Point3d(x, y, z);
            }
        }

        public double A
        {
            get { return _normal.X; } 
        }
        public double B
        {
            get { return _normal.Y; }
        }
        public double C
        {
            get { return _normal.Z; }
        }
        public double D
        {
            get
            {
                Point3d p = _point;
                Vector3d v = _normal;
                return -v.X * p.X - v.Y * p.Y - v.Z * p.Z;
            }
        }
    }
}
