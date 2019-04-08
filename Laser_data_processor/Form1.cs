using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Threading;
using System.Globalization;
using robot_vector;


/* 개발 사항
1. 두번째 세번째는 대표 방향만 나타낼 것

2. 마지막 두 줄(플랜지 방향)은 E 30 N 40 U20 이런식으로 나타낼것

3. 만약 E 70 N 20 U 이런 방식이면, N 20 E 20 U 이렇게 문자 순서를 바꾸고  90도 중 작은 각도로 나타낼 것

*/
namespace Laser_data_processor
{
    public partial class Form1 : Form
    {
        SerialPort _SP = new SerialPort();
        System.Timers.Timer _timer = new System.Timers.Timer();

        static string valEncoder_Laser = "";
        static string data_recieve = "";
        int point_num = 0;
        Pipe pipe1;
        Pipe pipe2;
        Matrix4d inv_H; // Transformation form Laser coordinate to pipe coordinate.
        public Form1()
        {
            InitializeComponent();
            this.Load += Form1_Load; ;
            _SP.DataReceived += _SP_DataReceived;

            //_timer.Elapsed += _timer_Elapsed;
            //_timer.Interval = 100;
            //_timer.Start();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            var ports = SerialPort.GetPortNames();
            cmbComPort.DataSource = ports;


        }

#if false
        //private void _timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        //{
        //    try
        //    {
        //        if (flag_encoder_laser == true)
        //        {
        //           // SetTextBox(valEncoder_Laser);
        //            flag_encoder_laser = false;
        //        }
        //    }
        //    catch (Exception ex)
        //    {
        //        Console.WriteLine(ex.Message);
        //    }
        //}
#endif

        delegate void SetTextCallback(string text);

        private void SetTextBox(string text)
        {
            if (this.tbPoint1.InvokeRequired)
            {
                //이코드가 없으면 크로스 스레드 작업 에러가 뜬다.
                SetTextCallback d = new SetTextCallback(SetTextBox);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                switch (point_num)
                {
                    case 1:
                        tbPoint1.Text = text;
                        break;
                    case 2:
                        tbPoint2.Text = text;
                        break;
                    case 3:
                        tbPoint3.Text = text;
                        break;
                    case 4:
                        tbPoint4.Text = text;
                        break;
                    case 5:
                        tbPoint5.Text = text;
                        break;
                    case 6:
                        tbPoint6.Text = text;
                        break;

                    default:
                        Console.WriteLine("default case");
                        break;
                }
                //  this.tbPoint1.Text = text;
            }
        }

        private void _SP_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                data_recieve = _SP.ReadLine();

                if (data_recieve.Length > 1)
                {
                    valEncoder_Laser = data_recieve.Substring(1, data_recieve.Length - 1);
                    // tbPoint.Text = valEncoder_Laser; 이 코드를 넣으면 크로스스레드 에러뜬다.

                    // 즉 GUI업데이트를 하려면 
                    // 1. 일단 스레드 예외처리는 필수이다.

                    List<string> list = valEncoder_Laser.Split(',').ToList<string>();
                    string s1 = list[0];
                    string s2 = list[1];
                    string s3 = list[2];

                    double angle1 = double.Parse(list[0]);
                    double angle2 = double.Parse(list[1]);
                    double L_dist = double.Parse(list[2]);

                    Point3d p1 = func(angle1, angle2, L_dist);
                    Console.WriteLine(p1);
                    string temp = string.Format("  {0,7:#.##} , {1,7:#.##} , {2,7:#.##}  ", p1.X, p1.Y, p1.Z);
                    SetTextBox(temp);

                    //      flag_encoder_laser = true;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }
        static double D2R(double degrees) //Degree to Radian
        {
            double angle = Math.PI * degrees / 180.0;
            return angle;
        }
        static double R2D(double radian) //Radian to Degree
        {
            double angle = radian * 180 / Math.PI;
            return angle;
        }
        static Point3d func(double motor1_angle, double motor2_angle, double laser_distance)
        {
            //********  D-H Table   ***************
            Matrix4d A1 = T(0, 0, 0, 0);
            Matrix4d A2 = T(-motor1_angle, 0, 0, 90);
            Matrix4d A3 = T(-motor2_angle, 6, 62, 90);
            Matrix4d A4 = T(0, -(59.2 + laser_distance), 0, 0);

            Matrix4d M = A1 * A2 * A3 * A4;
            //  Console.WriteLine(M);

            Point3d p = new Point3d(M[0, 3], M[1, 3], M[2, 3]);
            return p;
        }

        static Matrix4d T(double theta, double d, double a, double alpha)
        {
            double[] row1 = { Math.Cos(D2R(theta)), -Math.Sin(D2R(theta)) * Math.Cos(D2R(alpha)), Math.Sin(D2R(theta)) * Math.Sin(D2R(alpha)), a * Math.Cos(D2R(theta)) };
            double[] row2 = { Math.Sin(D2R(theta)), Math.Cos(D2R(theta)) * Math.Cos(D2R(alpha)), -Math.Cos(D2R(theta)) * Math.Sin(D2R(alpha)), a * Math.Sin(D2R(theta)) };
            double[] row3 = { 0, Math.Sin(D2R(alpha)), Math.Cos(D2R(alpha)), d };
            double[] row4 = { 0, 0, 0, 1 };

            Matrix4d M = new Matrix4d(row1, row2, row3, row4);
            //Console.WriteLine(M);
            return M;

        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                string portName = cmbComPort.Text;
                int Baud = 9600;

                _SP.PortName = portName;
                _SP.BaudRate = Baud;
                _SP.Parity = Parity.None;
                _SP.DataBits = 8;
                _SP.StopBits = StopBits.One;
                _SP.WriteTimeout = 30;
                _SP.ReadTimeout = 30;
                _SP.DtrEnable = true;
                _SP.RtsEnable = true;

                _SP.Open();

                if (_SP.IsOpen)
                {
                    if (data_recieve.Length > 3)
                    {
                        Console.WriteLine(data_recieve);
                    }
                }


            }
            catch (Exception ex)
            {
                Console.Write(ex.Message);
            }
        }

        //Turn on the Laser
        private void button2_Click(object sender, EventArgs e)
        {
            if (_SP.IsOpen)
            {
                byte[] frame = new byte[2];
                frame[0] = 0x02;
                frame[1] = 0x01;

                _SP.Write(frame, 0, frame.Length);
                //  Thread.Sleep(100);
            }
        }

        //Turn off the laser
        private void button3_Click(object sender, EventArgs e)
        {
            if (_SP.IsOpen)
            {
                byte[] frame = new byte[2];
                frame[0] = 0x02;
                frame[1] = 0x03;

                _SP.Write(frame, 0, frame.Length);
                //  Thread.Sleep(100);
            }
        }

        private void Send_Requestion()
        {
            if (_SP.IsOpen)
            {
                byte[] frame = new byte[2];
                frame[0] = 0x03;
                frame[1] = 0x02;
                _SP.Write(frame, 0, frame.Length);// 데이터 주세요 요청
                //한가지 발견한 사실은 요청한후 5초뒤에 데이터가 프로그램에 도착하는데
                //데이터가 TB에 표시된 이후로는 이 함수 밑으로의 함수는 더 이상 동작을 하지 않는다.
            }
        }
        // Request to Rpi3 that encoder1,2 + Laser distance data
        private void btn_Point_Pick(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 1;
        }

        private void button5_Click(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 2; //second tbPoint;
        }

        private void button6_Click(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 3; //second tbPoint;
        }

        private void button9_Click(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 4; //second tbPoint;
        }

        private void button8_Click(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 5; //second tbPoint;
        }

        private void button7_Click(object sender, EventArgs e)
        {
            Send_Requestion();
            point_num = 6; //second tbPoint;
        }

        private Point3d Get_Point_from_Textbox(TextBox textbox)
        {
            List<string> list = textbox.Text.Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries).ToList<string>();

            double X = double.Parse(list[0]);
            double Y = double.Parse(list[1]);
            double Z = double.Parse(list[2]);
            Point3d point = new Point3d(X, Y, Z);
            return point;
        }

        //pipe 직경, 중심점 계산
        private void button10_Click(object sender, EventArgs e)
        {
            try
            {
                Point3d p1 = null;
                Point3d p2 = null;
                Point3d p3 = null;


                p1 = Get_Point_from_Textbox(tbPoint1);
                p2 = Get_Point_from_Textbox(tbPoint2);
                p3 = Get_Point_from_Textbox(tbPoint3);


                if ((p1 != null) && (p2 != null) && (p3 != null))
                {
                    pipe1 = new Pipe(p1, p2, p3);
                    pipe1.Set_Info_To_TextBox(tbCenter_P1, tbDiameter);
                    //텍스트박스에 중점의 좌표와 직경을 표시해준다.
                }

                Console.WriteLine();
                Console.WriteLine("pipe1's center point(Laser Origin)");
                SHOW_N_S_W_E(pipe1._center_point);

                Console.WriteLine("pipe1's normal vector");
                SHOW_N_S_W_E(pipe1._normal_vector);
                //********************************

                double[] vec1 = { 1, 0, 0, pipe1._center_point.X };
                double[] vec2 = { 0, 1, 0, pipe1._center_point.Y };
                double[] vec3 = { 0, 0, 1, pipe1._center_point.Z };
                double[] vec4 = { 0, 0, 0, 1 };

                Matrix4d translation = new Matrix4d(vec1, vec2, vec3, vec4, (int)MyEnum.row_type);
                Console.WriteLine("translation to pipe1's center point");
                Console.WriteLine(translation);
                Console.WriteLine();
                Console.WriteLine("Inverse of translation");
                Console.WriteLine(translation.Inverse());
                inv_H = translation.Inverse();

            }
            catch (Exception ex)
            {
                MessageBox.Show("세 개의 점 데이터가 필요합니다.");
                Console.WriteLine(ex.Message);
            }
        }

        private void button11_Click(object sender, EventArgs e)
        {
            try
            {
                Point3d p4 = null;
                Point3d p5 = null;
                Point3d p6 = null;


                p4 = Get_Point_from_Textbox(tbPoint4);
                p5 = Get_Point_from_Textbox(tbPoint5);
                p6 = Get_Point_from_Textbox(tbPoint6);


                if ((p4 != null) && (p5 != null) && (p6 != null))
                {
                    pipe2 = new Pipe(p4, p5, p6);
                    pipe2.Set_Info_To_TextBox(tbCenter_P2);
                }

                Console.WriteLine("pipe2's center point (Laser Origin)");
                SHOW_N_S_W_E(pipe2._center_point);
                //Console.WriteLine(pipe2._center_point);

                Console.WriteLine("pipe2's normal vector");
                SHOW_N_S_W_E(pipe2._normal_vector);
                //Console.WriteLine(pipe2_normal);
            }
            catch (Exception ex)
            {
                MessageBox.Show("세 개의 점 데이터가 필요합니다.");
                Console.WriteLine(ex.Message);
            }
        }

        private void button12_Click(object sender, EventArgs e)
        {
            try
            {
                double distance = Math.Sqrt(Math.Pow((pipe1._center_point.X - pipe2._center_point.X), 2) +
                                            Math.Pow((pipe1._center_point.Y - pipe2._center_point.Y), 2) +
                                            Math.Pow((pipe1._center_point.Z - pipe2._center_point.Z), 2));
                tbDistance.Text = distance.ToString("0.##");


                //바뀐 좌표계 U, N, E에서의 파이프2의 중심좌표
                Console.WriteLine();
                Console.WriteLine("Distance between pipe1 and pipe2 (mm)");
                Point3d cp = pipe2._center_point.Copy();
                Vector4d pipe2_CenterPoint = new Vector4d(cp.X, cp.Y, cp.Z, 1);
                Vector4d res_cp = inv_H.Mult(pipe2_CenterPoint);
                Point3d res = new Point3d(res_cp.X, res_cp.Y, res_cp.Z);
                string temp = string.Format("  {0,7:0.##} , {1,7:0.##} , {2,7:0.##}  ", res_cp.X, res_cp.Y, res_cp.Z);
                tb_pipe2_cp_by_pipe1.Text = temp;
                Console.WriteLine("pipe2's center point (pipe1's view point)");
                SHOW_N_S_W_E(res);
            }
            catch (Exception)
            {
                MessageBox.Show("두 개의 파이프에 대한 데이터가 부족합니다.");
            }
        }

        private Matrix3d Rotation_From_Two_Vector(Vector3d v1, Vector3d v2)
        {
            Vector3d w = v1.Cross(v2).Normalized;
            Matrix3d K = w.Make_Skew_Matrix();
            double cos_theta = (v1.Dot(v2) / v1.Norm) / v2.Norm;
            double theta = Math.Acos(cos_theta); //theta값은 현재 라디안이다. C#의 삼각함수는 모두 라디안이 기준이다.

            Matrix3d R = Matrix3d.Identity() + Math.Sin(theta) * K + (1 - Math.Cos(theta)) * K * K;
            return R;
        }
        private double[] Transfromation_to_Euler_angle(Matrix3d R)
        {
            double[] angle = { 0, 0, 0 };
            // alpha, beta, gamma sequence

            //beta
            angle[1] = Math.Atan2(Math.Sqrt(R[2, 0] * R[2, 0] + R[2, 1] * R[2, 1]), R[2, 2]);
            double beta = angle[1]; //라디안값이다

            //alpha
            angle[0] = Math.Atan2(R[1, 2] / Math.Sin(beta), R[0, 2] / Math.Sin(beta));
            double alpha = angle[0];

            angle[2] = Math.Atan2(R[2, 1] / Math.Sin(beta), -R[2, 0] / Math.Sin(beta));

            //시간이 되면 try catch 구문도 집어넣자. 베타가 0이 되거나 180이 될 수 있으니까
            return angle;
        }
        private void SHOW_N_S_W_E(Point3d p1)
        {
            Point3d p = p1.Copy();
            char[] dir = { 'E', 'N', 'U' };
            if (p.X < 0)
            {
                dir[0] = 'W';
                p.X = -p.X;
            }
            if (p.Y < 0)
            {
                dir[1] = 'S';
                p.Y = -p.Y;
            }
            if (p.Z < 0)
            {
                dir[2] = 'D';
                p.Z = -p.Z;
            }
            Console.WriteLine(dir[0] + ": {0:0.##}, " + dir[1] + ": {1:0.##}, " + dir[2] + ": {2:0.##}", p.X, p.Y, p.Z);
        }
        private void SHOW_N_S_W_E(Vector3d v1)
        {
            Vector3d v = v1.Copy();
            char[] dir = { 'E', 'N', 'U' };
            if (v.X < 0)
            {
                dir[0] = 'W';
                v.X = -v.X;
            }
            if (v.Y < 0)
            {
                dir[1] = 'S';
                v.Y = -v.Y;
            }
            if (v.Z < 0)
            {
                dir[2] = 'D';
                v.Z = -v.Z;
            }
            Console.WriteLine(dir[0] + ": {0:0.##}, " + dir[1] + ": {1:0.##}, " + dir[2] + ": {2:0.##}", v.X, v.Y, v.Z);
            Console.WriteLine();
            Console.WriteLine("단면(수직벡터)의 회전각도");
            double angle_EN = R2D(Math.Atan2(v.Y, v.X));
            double angle_NU = R2D(Math.Atan2(v.Z, Math.Sqrt(v.X * v.X + v.Y * v.Y)));
            Console.WriteLine(dir[0] + " {0:0.##} " + dir[1] + " {1:0.##} " + dir[2], angle_EN, angle_NU);

        }
        //private void SHOW_N_S_W_E(Point3d p1, StreamWriter file)
        //{
        //    Point3d p = p1.Copy();
        //    char[] dir = { 'E', 'N', 'U' };
        //    if (p.X < 0)
        //    {
        //        dir[0] = 'W';
        //        p.X = -p.X;
        //    }
        //    if (p.Y < 0)
        //    {
        //        dir[1] = 'S';
        //        p.Y = -p.Y;
        //    }
        //    if (p.Z < 0)
        //    {
        //        dir[2] = 'D';
        //        p.Z = -p.Z;
        //    }
        //    file.WriteLine(dir[0] + " {0:0.##} " + dir[1] + " {1:0.##} " + dir[2] + " {2:0.##}", p.X, p.Y, p.Z);
        //}
        private void SHOW_N_S_W_E(Vector3d v1, StreamWriter file)
        {
            Vector3d v = v1.Copy();
            //Vector3d v = -v1.Copy();
          // v1.copy() 앞의 -를 다시 없애서 원래 코드로 다시 복귀했다.
          // 파이프 단면 방향을 다시 반전 시켜주기 위해서다.

            //또 다른 조건으로는, E,N,U 세 축 모두 방향을 표시할 게 아니라 가장 
            //성분이 큰 방향 하나만 나타낸다.
            // 간혹 E 45 N 이런 식으로 사이 값이 나온다고 하는데, 
            // 먼저 테스트로 제일 성분 큰 방향 한 개 방향만 나타내도록 하자.


            char[] dir = { 'E', 'N', 'U' };
            if (v.X < 0)
            {
                dir[0] = 'W';
                v.X = -v.X;
            }
            if (v.Y < 0)
            {
                dir[1] = 'S';
                v.Y = -v.Y;
            }
            if (v.Z < 0)
            {
                dir[2] = 'D';
                v.Z = -v.Z;
            }


            if (v.X > v.Y && v.X > v.Z)
            {
                file.WriteLine(dir[0]);
            }
            if (v.Y > v.X && v.Y > v.Z)
            {
                file.WriteLine(dir[1]);
            }
            if (v.Z > v.X && v.Z > v.Y)
            {
                file.WriteLine(dir[2]);
            }

            //file.WriteLine(dir[0] + " {0:0.##} " + dir[1] + " {1:0.##} " + dir[2] + " {2:0.##}", v.X, v.Y, v.Z);
          //  Console.WriteLine(dir[0] + " {0:0.##} " + dir[1] + " {1:0.##} " + dir[2] + " {2:0.##}", v.X, v.Y, v.Z);

        }
        private void SHOW_N_S_W_E_Plane_angle(Vector3d v1, StreamWriter file)
        { //각도 2개 모두 기록
            Vector3d v = v1.Copy();
            char[] dir = { 'E', 'N', 'U' };
            if (v.X < 0)
            {
                dir[0] = 'W';
                v.X = -v.X;
            }
            if (v.Y < 0)
            {
                dir[1] = 'S';
                v.Y = -v.Y;
            }
            if (v.Z < 0)
            {
                dir[2] = 'D';
                v.Z = -v.Z;
            }
            double angle_EN = R2D(Math.Atan2(v.Y, v.X));
            double angle_NU = R2D(Math.Atan2(v.Z, Math.Sqrt(v.X * v.X + v.Y * v.Y)));

            if (angle_EN < 45)
            { //45도 보다 작으면 E num N 순으로 작성
                file.WriteLine(dir[0] + " " + Convert.ToInt32(angle_EN) + " " + dir[1] + " " + Convert.ToInt32(angle_NU) + " " + dir[2]);
            }
            else
            { // 45도 보다 크면 N 90-num E 순으로 순서를 한 번 바꿔준다.
                angle_EN = 90 - angle_EN;
                file.WriteLine(dir[1] + " " + Convert.ToInt32(angle_EN) + " " + dir[0] + " " + Convert.ToInt32(angle_NU) + " " + dir[2]);
            }
                //file.WriteLine(dir[0] + " {0:0.##} " + dir[1] + " {1:0.##} " + dir[2], angle_EN, angle_NU);
        }
        private void SHOW_N_S_W_E_Plane_angle_ver2(Vector3d v1, StreamWriter file)
        { // 각도 1개만 기록
            Vector3d v = v1.Copy();
            char[] dir = { 'E', 'N', 'U' };
            if (v.X < 0)
            {
                dir[0] = 'W';
                v.X = -v.X;
            }
            if (v.Y < 0)
            {
                dir[1] = 'S';
                v.Y = -v.Y;
            }
            if (v.Z < 0)
            {
                dir[2] = 'D';
                v.Z = -v.Z;
            }
            double angle_EN = R2D(Math.Atan2(v.Y, v.X));
            double angle_NU = R2D(Math.Atan2(v.Z, Math.Sqrt(v.X * v.X + v.Y * v.Y)));
            file.WriteLine(dir[0] + " " + Convert.ToInt32(angle_EN) + " " + dir[1]);
        }

        //Save Text file button
        private void button13_Click(object sender, EventArgs e)//v1은  E, N, U 축중 가장 큰 방향 한개만(W,S,D도 가능) 저장하는 버전
        {

            try
            {
                if (pipe1 == null || pipe2 == null) throw new ArgumentNullException();

                string dir = "D:\\PDMS_OUTPUT\\ACEENG\\MODEL_DATA";
                System.IO.Directory.CreateDirectory(dir);
                string path = dir + "\\" + DateTime.Now.ToString("yyyy-MM-dd") + ".txt";
                path = getNextFileName(path);

                MessageBox.Show("아래 경로에 파일이 저장됩니다.\n"+"저장경로: " + dir+ "\n"+"파일이름: "+ Path.GetFileName(path));

                using (StreamWriter sw = new StreamWriter(path))
                {
                    //    SaveFileDialog saveFileDialog1 = new SaveFileDialog();  // 대화상자 생성코드
                    //        saveFileDialog1.FileName = "DefaultOutputName.txt";
                    //               saveFileDialog1.Filter = "Text File | *.txt";
                    //               if (saveFileDialog1.ShowDialog() == DialogResult.OK)
                    //             {
                    //Stream s = File.Open(saveFileDialog1.FileName, FileMode.CreateNew);
                    //StreamWriter sw = new StreamWriter(s);
    
                    Point3d cp = pipe2._center_point.Copy();
                    Vector4d res_cp = inv_H.Mult(new Vector4d(cp.X, cp.Y, cp.Z, 1));
                    Point3d res = new Point3d(res_cp.X, res_cp.Y, res_cp.Z);
                    int cnt = 0; //End좌표중 부호가 양수인 갯수를 기록
                    if (res.X > 0) cnt++;
                    if (res.Y > 0) cnt++;
                    if (res.Z > 0) cnt++;

                    sw.WriteLine(Convert.ToInt32(pipe1._diameter)); //첫째 줄 : 파이프의 직경

                    if (cnt >= 2) { // 양수가 많으면 정상대로 기록
                        SHOW_N_S_W_E(pipe1._normal_vector, sw); //둘째 줄: Pipe1 Start dir
                        SHOW_N_S_W_E(pipe2._normal_vector, sw); //셋째 줄: Pipe2 End dir
                    }
                    else
                    {//음수가 많으면 파이프 1,2 기록순서를 스왑해준다.
                        SHOW_N_S_W_E(pipe2._normal_vector, sw); //둘째 줄: Pipe1 Start dir
                        SHOW_N_S_W_E(pipe1._normal_vector, sw); //셋째 줄: Pipe2 End dir
                    }

                    sw.WriteLine("0"); //넷째 줄: Start 좌표: 0,0,0

                    sw.WriteLine("0"); //다섯째 줄: 0 무조건 입력

                    if (cnt >= 2)
                    { //양수가 많으므로 정상적으로 기록
                        sw.WriteLine(Convert.ToInt32(res.X) + "," + Convert.ToInt32(res.Y) + "," + Convert.ToInt32(res.Z)); //여섯째 줄: End좌표
                    }
                    else
                    {
                     //음수가 많으므로 파이프1과 파이프2의 관점을 스왑해줌
                     //그래서 부호를 바꿔주어야 한다.
                        sw.WriteLine(Convert.ToInt32(-res.X) + "," + Convert.ToInt32(-res.Y) + "," + Convert.ToInt32(-res.Z)); //여섯째 줄: End좌표
                    }


                    double distance = Math.Sqrt(Math.Pow((pipe1._center_point.X - pipe2._center_point.X), 2) +
                                                   Math.Pow((pipe1._center_point.Y - pipe2._center_point.Y), 2) +
                                                   Math.Pow((pipe1._center_point.Z - pipe2._center_point.Z), 2));
                    sw.WriteLine(Convert.ToInt32(distance)); //일곱번째 줄: Dist: 두 중점 사이의 거리

                    if (cnt >= 2)//양수가 많으므로 정상적으로 기록
                    {
                        SHOW_N_S_W_E_Plane_angle(pipe1._normal_vector, sw); //  Start 단면
                        SHOW_N_S_W_E_Plane_angle(pipe2._normal_vector, sw); //End 단면
                        //sw.WriteLine("0,0,0"); // 여덟번째 줄: Start 단면;
                        //sw.WriteLine("0,0,0"); //마지막 줄 : end 단면
                    }

                    else //음수가 많으므로 파이프 1과 파이프2의 기록 순서 변경!
                    {
                         SHOW_N_S_W_E_Plane_angle(pipe2._normal_vector, sw); //  Start 단면
                         SHOW_N_S_W_E_Plane_angle(pipe1._normal_vector, sw); //End 단면
                        //sw.WriteLine("0,0,0"); // 여덟번째 줄: Start 단면;
                        //sw.WriteLine("0,0,0"); //마지막 줄 : end 단면
                    }
                    
                    sw.Dispose();
                    sw.Close();
                }
            }

            catch (Exception ex)
            {
                MessageBox.Show("pipe에 대한 데이터가 부족합니다.");
            }
        } 


        static private string getNextFileName(string fileName)
        {
            string extension = Path.GetExtension(fileName);
            

            int i = 0;
            while (File.Exists(fileName))
            {
                if (i == 0)
                    fileName = fileName.Replace(extension, "(" + ++i + ")" + extension);
                else
                    fileName = fileName.Replace("(" + i + ")" + extension, "(" + ++i + ")" + extension);
            }

            return fileName;
        }

        private void button14_Click(object sender, EventArgs e)
        {
            if (_SP.IsOpen)
            {
                // Read Laser

                float MaxRange = 5000;
                float Percentage = (float)numValue1.Value / 100; // 100 %
                Int16 num = Convert.ToInt16(((MaxRange - 100) * Percentage + 100));
                byte[] value = BitConverter.GetBytes(num);
                byte[] frame = new byte[4];
                frame[0] = 0x01; // Id code
                frame[1] = 0x01; // motor 1
                frame[2] = value[1];
                frame[3] = value[0];
                _SP.Write(frame, 0, frame.Length);
                Thread.Sleep(100);
            }

        }

        private void button15_Click(object sender, EventArgs e)
        {
            if (_SP.IsOpen)
            {
                // Read Laser

                float MaxRange = 2000;
                float Percentage = (float)numValue2.Value / 100; // 100 %
                Int16 num = Convert.ToInt16(((MaxRange - 100) * Percentage + 100));
                byte[] value = BitConverter.GetBytes(num);
                byte[] frame = new byte[4];
                frame[0] = 0x01; // Id code
                frame[1] = 0x02; // motor 2
                frame[2] = value[1];
                frame[3] = value[0];
                _SP.Write(frame, 0, frame.Length);
                Thread.Sleep(100);
            }
        }
    }
}
