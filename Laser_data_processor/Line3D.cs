using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot_vector
{
    class Line3d
    {
        private Point3d _point;
        private Vector3d _dir;

        public Line3d()
        {
            _point = new Point3d();
            _dir = new Vector3d(1, 0, 0);
        }

        public Line3d(Point3d p , Vector3d v)
        {
            _point = p.Copy();
            _dir = v.Copy();
        }

        public Point3d Point
        {
            get { return _point.Copy(); }
            set { _point = value.Copy(); }
        }
        public Vector3d Direction
        {
            get { return _dir.Copy(); }
            set { _dir = value.Copy(); }
        }
        
        public double DistanceTo(Point3d p)
        {
            return p.DistanceTo(this);
        }
    }
}
