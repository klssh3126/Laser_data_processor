using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Laser_data_processor;

namespace robot_vector
{
    class Pipe
    {
        private Point3d center_point;
        private double diameter;
        private Vector3d normal_vector;


        public Pipe(Point3d p1, Point3d p2, Point3d p3)
        {
                Point3d M = (p1 + p2) / 2;
                Point3d N = (p1 + p3) / 2;

                Vector3d n12 = new Vector3d(p1, p2);
                Vector3d n13 = new Vector3d(p1, p3);

                Plane3d s0 = new Plane3d(p1, p2, p3);
                Plane3d s1 = new Plane3d(M, n12);
                Plane3d s2 = new Plane3d(N, n13);

                object obj1 = s0.IntersectionWith(s1, s2);

                if (obj1.GetType() == typeof(Point3d))
                {
                    center_point = (Point3d)obj1;
                    double R1 = Math.Sqrt(Math.Pow((p1.X - center_point.X), 2) + Math.Pow((p1.Y - center_point.Y), 2) + Math.Pow((p1.Z - center_point.Z), 2));
                    diameter = 2 * R1;
                }

                Vector3d o_p_vector = new Vector3d(center_point);
                normal_vector = n12.Cross(n13).Copy().Normalized;

                if (o_p_vector.Dot(normal_vector) > 0) //불가능한 파이브 방향이다.
                    normal_vector = -normal_vector; //파이프의 노멀벡터의 부호를 레이저가 있는 부호면으로 결정하기 위한 코드
        }

        public void Set_Info_To_TextBox(TextBox tbCenterPoint, TextBox tbDiameter = null)
        {
            string temp = string.Format("  {0,7:#.##} , {1,7:#.##} , {2,7:#.##}  ", center_point.X, center_point.Y, center_point.Z);
           tbCenterPoint.Text = temp;

            if (tbDiameter != null)
            {
                tbDiameter.Text = diameter.ToString("#.##");
            }
        }

        public Point3d _center_point
        {
            get { return center_point; }
            set { center_point = value; }
        }
        public Vector3d _normal_vector
        {
            get { return normal_vector; }
            set { normal_vector = value; }
        }
        public double _diameter
        {
            get { return diameter; }
            set { diameter = value; }
        }
    }
}
