using devDept.Eyeshot;
using devDept.Eyeshot.Entities;
using devDept.Eyeshot.Translators;
using devDept.Eyeshot.Triangulation;
using devDept.Geometry;
using devDept.Graphics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Forms;
using System.Windows.Input;
using GeoUtility;
using Color = System.Drawing.Color;
using System.Diagnostics;

namespace GraphView
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private const string m_DefaultMaterial = "714FDFD0-289B-48C1-9C28-7D327CD84C44";
        private const string m_MeshLayer = "BCBC59FA-11C2-4B69-9842-8A3F487AE51A";
        private const string m_PointCloudLayer = "DA681219-991E-406D-9408-0D5A7DAFEB11";
        private Dictionary<string,int> m_LayerList;
        private List<Mesh> m_MeshList;
        private List<Point3D> m_PointList;
        private Random m_Random = new Random();
        private Vertex[] m_FromTo = new Vertex[2];

        private class Location
        {
            public Point3D o;
            public Vector3D x;
            public Vector3D y;
            public Vector3D z;
            public Location(Point3D pt, Quaternion qua)
            {
                double[,] m = new double[4, 4];
                qua.ToMatrix(out m);
                x = new Vector3D(1, 0, 0);
                y = new Vector3D(0, 1, 0);
                z = new Vector3D(0, 0, 1);
                Transformation t = new Transformation(m);
                x.TransformBy(t);
                y.TransformBy(t);
                z.TransformBy(t);
                o = pt;
            }
            public void Show(ViewportLayout eyeshot,int layer)
            {
                Joint j = new Joint(o, 0.5, 1);
                Mesh xx = Mesh.CreateArrow(o, x, 0.2, 5, 0.35, 1, 16, Mesh.natureType.Smooth, Mesh.edgeStyleType.Free);
                Mesh yy = Mesh.CreateArrow(o, y, 0.2, 5, 0.35, 1, 16, Mesh.natureType.Smooth, Mesh.edgeStyleType.Free);
                Mesh zz = Mesh.CreateArrow(o, z, 0.2, 5, 0.35, 1, 16, Mesh.natureType.Smooth, Mesh.edgeStyleType.Free);

                eyeshot.Entities.Add(j, layer, Color.Black);
                eyeshot.Entities.Add(xx, layer, Color.Red);
                eyeshot.Entities.Add(yy, layer, Color.Green);
                eyeshot.Entities.Add(zz, layer, Color.Blue);
                //eyeshot.Invalidate();
            }
        }

        private Color RandomColor()
        {
            int r = m_Random.Next(1, 255); 
            int g = m_Random.Next(1,255);
            int b = m_Random.Next(1, 255); 
            return Color.FromArgb(r,g,b);
        }

        private void CheckInterference(bool bReloc=false)
        {
            if(bReloc)
                eyeshot.Entities.Clear();

            Color c0 = Color.Blue;
            Color c =bReloc?Color.FromArgb(180,Color.Yellow): c0;
            List<Circle> circles = new List<Circle>();
            circles.Add(new Circle(new Point3D(-30, 35, 0), 7));
            circles.Add(new Circle(new Point3D(30, 35, 0), 7));
            circles.Add(new Circle(new Point3D(-38, 12, 0), 6));
            circles.Add(new Circle(new Point3D(38, 12, 0), 6));
            circles.Add(new Circle(new Point3D(-15, 43, 0), 6));
            circles.Add(new Circle(new Point3D(15, 43, 0), 6));
            circles.Add(new Circle(new Point3D(-20, -33, 0), 6));
            circles.Add(new Circle(new Point3D(20, -33, 0), 6));
            List<Joint> joints = new List<Joint>();
            double radius = 0.5;
            joints.Add(new Joint(new Point3D(-8, 9, 0),radius,1));
            joints.Add(new Joint(new Point3D(8, 9, 0), radius, 1));
            joints.Add(new Joint(new Point3D(-3.5, 11.1, 0), radius, 1));
            joints.Add(new Joint(new Point3D(3.5, 11.1, 0), radius, 1));
            for (int i = 0; i < circles.Count; i++)
            {
                eyeshot.Entities.Add(circles[i], 0, c);
            }
            for (int i = 0; i < joints.Count; i++)
            {
                eyeshot.Entities.Add(joints[i], 0, c);
            }


            Circle c1 = new Circle(new Point3D(0.000000000, -20.0000000, 0.000000000), 6);
            Circle c1_ = new Circle(new Point3D(0.000000000, -20.0000000, 0.000000000), 9);
            Circle c2 = new Circle(new Point3D(25.0000000, 15.0000000, 0.000000000), 6);
            Circle c2_ = new Circle(new Point3D(25.0000000, 15.0000000, 0.000000000), 9);
            Circle c3 = new Circle(new Point3D(-25.0000000, 15.0000000, 0.000000000), 6);
            Circle c3_ = new Circle(new Point3D(-25.0000000, 15.0000000, 0.000000000), 9);

            eyeshot.Entities.Add(c1, 0, Color.Red);
            eyeshot.Entities.Add(c1_, 0, Color.Red);
            eyeshot.Entities.Add(c2, 0, Color.Red);
            eyeshot.Entities.Add(c2_, 0, Color.Red);
            eyeshot.Entities.Add(c3, 0, Color.Red);
            eyeshot.Entities.Add(c3_, 0, Color.Red);


            if(bReloc)
            {
                Transformation trans = new Transformation();
                trans.Rotation(3.14, Vector3D.AxisZ);
                trans.Translation(new Vector3D(0, 5, 0));
                for (int i = 0; i < circles.Count; i++)
                {
                    circles[i].TransformBy(trans);
                    eyeshot.Entities.Add(circles[i], 0, c0);
                }
                for (int i = 0; i < joints.Count; i++)
                {
                    joints[i].TransformBy(trans);
                    eyeshot.Entities.Add(joints[i], 0, c0);
                }
                eyeshot.Invalidate();
                eyeshot.SetView(viewType.Bottom);
                eyeshot.ZoomFit();
                

            }

        }

        private void TestSurf()
        {
            const int nU = 4, nV = 4;
            const int dU = 3, dV = 3;
            Point4D[,] pts4 = new Point4D[nU, nV];
            Random rnd = new Random();
            List<Point3D> pts3 = new List<Point3D>();
            double dx = 0.8, dy = 1.2;
            for (int i = 0; i < nU; i++)
            {
                for (int j = 0; j < nV; j++)
                {
                    var p = new Point3D(dx * j, dy * i, 2.0 * rnd.Next(-1, 1));
                    pts3.Add(p);
                    pts4[i, j] = new Point4D(p);
                }
            }
            Surface surf = new Surface(dU, Surface.UniformKnotVector(dU, nU), dV, Surface.UniformKnotVector(dU, nV), pts4);
            surf.ShowCurvature = true;
            surf.ShowControl = true;
            eyeshot.Entities.Add(new PointCloud(pts3, 5), 0, Color.Red);
            eyeshot.Entities.Add(surf, 0, Color.Blue);
           
            eyeshot.Invalidate();
            eyeshot.ZoomFit();
        }

        private void CreateBox()
        {

#if DEBUG
            double a = 80, b = 4.2, c = 2.6, d = 0.8;//dx  dy dz chamfer

            List<Point3D> pts = new List<Point3D>();
            pts.Add(new Point3D(0, -d, 0));
            pts.Add(new Point3D(0, b + d, 0));
            pts.Add(new Point3D(0, b, d));
            pts.Add(new Point3D(0, b, c));
            pts.Add(new Point3D(0, 0, c));
            pts.Add(new Point3D(0, 0, d));
            pts.Add(new Point3D(0, -d, 0));

            LinearPath lp = new LinearPath(pts);
            Region r= new Region(lp);
            Mesh box=r.ExtrudeAsMesh(new Vector3D(a, 0, 0), 0.01, Mesh.natureType.Plain);
            
            Transformation trans = new Transformation();
            trans.Translation(new Vector3D(-a / 2, -0.1, -0.1));
            box.TransformBy(trans);

            eyeshot.Entities.Add(box, 0, Color.Red);


            eyeshot.Invalidate();
            eyeshot.SetView(viewType.Left);
            eyeshot.ZoomFit();

            WriteSTL wStl = new WriteSTL(eyeshot, "d:/ChamferBox.stl");
            wStl.DoWork();
#endif
           
        }

        private void TestDS()
        {
            BTree<int> btree = new BTree<int>();
            btree.root = new BiNode<int>(1);

            btree.root.left = new BiNode<int>(2);
            btree.root.right = new BiNode<int>(3);

            btree.root.left.left = new BiNode<int>(4);
            btree.root.left.right = new BiNode<int>(5);

            btree.root.left.right.right = new BiNode<int>(6);

            btree.PreTraverse().ForEach(a => Console.Write(a.ToString()+","));
            btree.MidTraverse().ForEach(a => Console.Write(a.ToString() + ","));
            btree.PostTraverse().ForEach(a => Console.Write(a.ToString() + ","));
            btree.LevelTraverse().ForEach(a => Console.Write(a.ToString() + ","));

            Debugger.Break();
        }

        private void TestMC()
        {
            eyeshot.GetGrid().Visible = true;

            eyeshot.GetGrid().Min = new Point3D(-5, -5);

            eyeshot.GetGrid().Max = new Point3D(+5, +5);

            eyeshot.GetGrid().Step = .5;

            ScalarField3D func = new ScalarField3D(ScalarFieldFunc);

            double resolution = 0.5;

            int span = 100;

            MarchingCubes mc = new MarchingCubes(new Point3D(-5, -5, -5), span, resolution, span, resolution, span, resolution, func);
           
            mc.IsoLevel = 4;

            eyeshot.DoWork(mc);

            Mesh res = mc.Result;

            res.FlipNormal();

            Mesh sphere = Mesh.CreateSphere(Math.Sqrt(mc.IsoLevel), 16, 32);

           // eyeshot.Entities.Add(sphere, 0, Color.FromArgb(170, Color.Blue));

            eyeshot.Entities.Add(res, 0, Color.FromArgb(170,Color.Red));

            eyeshot.Invalidate();

            eyeshot.ZoomFit();
          
        }

        private void TestSlice()
        {
            Mesh m = ImportModel("d:/cgal_boolean2.stl");
           // Mesh m = Mesh.CreateSphere(10, 16, 32);
            Point3D cp = new Point3D();
            foreach (var item in m.Vertices)
            {
                cp += item;
            }
            cp /= m.Vertices.Length;
            cp -= new Vector3D(0, 0, 4);

            Plane pln = new Plane(cp,-1*Vector3D.AxisZ);

            booleanFailureType res= m.CutBy(pln, true);

            Console.WriteLine(res.ToString());

            eyeshot.Entities.Add(m, 0, Color.Red);
            eyeshot.Invalidate();
            eyeshot.ZoomFit();

        }

        enum booleanType { Union, Difference, Intersection };

        double ScalarFieldFunc(double x,double y,double z)
        {      

            double fun1 = Math.Pow(x,2) + Math.Pow(y, 2) + Math.Pow(z, 2);

            double fun2= Math.Pow(x-5, 2) + Math.Pow(y, 2) + Math.Pow(z, 2);

            //fun2 = Math.Pow(x, 2) + Math.Pow(y, 2);

            //if (z < 0||z>5)
            //    fun2 = 100;
           // fun2 = Math.Min(fun2, 20);

            //double zero =  1e-6;

            booleanType type = booleanType.Difference;


            switch (type)
            {
                case booleanType.Union:
                    return Math.Min(fun1, fun2);
                case booleanType.Difference:
                    return Math.Max(fun1, -fun2);
                case booleanType.Intersection:
                    return Math.Max(fun1, fun2);
                default:
                    break;
            }

            return 0;           
        }

        private void TestHE()
        {
            string str = @"D:\cgal_boolean.stl";
            //str = "d:/tmp/plane3.stl";
            Mesh mesh = ImportModel(str);
            Polyhedron poly = new Polyhedron(mesh);
            var vv=poly.VV(poly.vList[0]);
            var vf= poly.VF(poly.vList[0]);
            var holes = poly.FindHoles();
            var veo = poly.VEO(poly.vList[0]);
            var vei = poly.VEI(poly.vList[0]);
            //double dist= poly.GDBase(poly.vList[1], poly.vList[2]);
            var src = poly.vList[m_Random.Next(1, poly.vList.Count - 1)];
            var dst = poly.vList[m_Random.Next(1, poly.vList.Count - 1)];

            {
                List<Vertex> path;
                double dist2 = poly.GD_OLD(src, dst, out path);
                eyeshot.Entities.Add(new Joint(src, 0.1, 1), 0, Color.White);
                eyeshot.Entities.Add(new Joint(dst, 0.1, 1), 0, Color.Black);
                for (int i = 0; i + 1 < path.Count; i++)
                {
                    Line l = new Line(path[i], path[i + 1]);
                    l.LineWeight = 6;
                    l.LineWeightMethod = colorMethodType.byEntity;
                    eyeshot.Entities.Add(l, 0, Color.Red);
                }
                Console.WriteLine("geo dist = " + dist2.ToString());
            }

            {
                List<Vertex> path;
                double dist2 = poly.GD(src, dst, out path);
                //eyeshot.Entities.Add(new Joint(src, 0.1, 1), 0, Color.White);
                //eyeshot.Entities.Add(new Joint(dst, 0.1, 1), 0, Color.Black);
                for (int i = 0; i + 1 < path.Count; i++)
                {
                    Line l = new Line(path[i], path[i + 1]);
                    l.LineWeight = 6;
                    l.LineWeightMethod = colorMethodType.byEntity;
                    eyeshot.Entities.Add(l, 0, Color.Yellow);
                }
                Console.WriteLine("geo dist = " + dist2.ToString());

            }

            Console.WriteLine(holes.Count.ToString() + " holes was found!");
            foreach (var hole in holes)
            {
                Color c = RandomColor();
                foreach (var he in hole)
                {
                    Edge e = new Edge(he);
                    Line l = new Line(poly.vList[e.V1], poly.vList[e.V2]);
                    l.LineWeight = 3;
                    l.LineWeightMethod = colorMethodType.byEntity;
                    eyeshot.Entities.Add(l, 0, c);
                }
            }
            eyeshot.Entities.Add(poly,0,Color.Green);
            eyeshot.Invalidate();
            eyeshot.ZoomFit();

         //   Debugger.Break();
        }

        WindingNumberRPD_Tree _wnTree;
       // CMesh _cmesh;

        private double BaseIO(Point3D p)
        {
            var t1 = DateTime.Now;
            var res = _wnTree.DetectProb(p);
            var t2 = DateTime.Now;
            DashBoard.Text = "TWN\t" + res.ToString("F5");
            DashBoard.Text += "\nCost\t" + (t2 - t1).Milliseconds + "ms";
            DashBoard.Text += "\nPoint\n" + p.ToString();
            return res;
        }

        private void RunIO_Tree(Point3D p=null)
        {
            bool showAll = true;
            bool showOut = true;

            int lb = -25, ub = 25;

            if (p == null)
            {
                p = new Point3D(m_Random.Next(lb, ub), m_Random.Next(lb, 0), m_Random.Next(-15, 5));
            }

            var res = BaseIO(p);

            double thres = 0.5;
            double r = 0.25;

            if(showAll)
            {
                eyeshot.Entities.Add(new Joint(p, r,1), 0, res < thres ? Color.DeepSkyBlue : Color.Red);
                return;
            }
            if (!showOut && res >= thres)
            {
                eyeshot.Entities.Add(new Joint(p, r, 1), 0, Color.Red);
                return;
            }
            if (showOut && res < thres)
            {
                eyeshot.Entities.Add(new Joint(p, r, 1), 0, Color.Blue);
                return;
            }
        }

        private void InitIO()
        {
            Task task1=new Task((Action)delegate
            {
                Mesh m = ImportModel("d:/test1w.stl");//cgal_boolean test  
                //m = Mesh.CreateSphere(25, 8, 16);  
                //_cmesh = new CMesh(m);
                //RPDTree rpd = _cmesh.GetComponent("RPD") as RPDTree;
                //_cmesh.RemoveComponent("a");
                //_cmesh.RemoveComponent("RPD");

                _wnTree = new WindingNumberRPD_Tree();
                _wnTree.Init(m);
            });

            task1.Start();

            Mesh ms = ImportModel("d:/test1w1.stl");
            eyeshot.Entities.Add(ms, 0, Color.FromArgb(255, Color.MediumPurple));

            task1.Wait();


        }

        private void RunIO_Force(Point3D p)
        {
            var m = _wnTree._mesh;

            {
                var hole = MeshUtility.FindHole(m.Triangles);
                foreach (var item in hole)
                {
                    Line l = new Line(m.Vertices[item.V1], m.Vertices[item.V2]);
                    l.LineWeight = 6;
                    l.LineWeightMethod = colorMethodType.byEntity;
                    eyeshot.Entities.Add(l, 0, Color.Crimson);
                }

                var tris = MeshUtility.TriangulateGreedy(hole, m);
                var rpd = RPDFactory.Create(RPDFactory.RPDType.WindingNumber);
                rpd.Init(m);
                double prob = (rpd as WindingNumberRPD).DetectProb(p);
                double prob2 = WindingNumberRPD.DetectProb(tris, p);

                DashBoard.Text += "\nWNa\t" + prob.ToString("F5");
                DashBoard.Text += "\nWNb\t" + prob2.ToString("F5");
                DashBoard.Text += "\nWNs\t" + (prob2 + prob).ToString("F5");
                //DashBoard.Text += "\nasin=" + Math.Asin(0.5).ToString("F5");
            }

        }

        private void TestIO()
        {
            

            InitIO();

            for (int i = 0; i < 10; i++)
            {
               RunIO_Tree();
            }



            //Point3D p = new Point3D(-1, -11, -12);
            //int lb = -25, ub = 25;
            //p = new Point3D(m_Random.Next(lb, ub), m_Random.Next(lb, 0), m_Random.Next(-15, 5));
      
            //DashBoard.Height += 50;
            //RunIO_Tree(p);
            //RunIO_Force(p);

            eyeshot.Invalidate();
            eyeshot.ZoomFit();
        }

        void TestDSAlgo()
        {
            {
                List<int> data = new List<int>() { 3, 5, 4, 2, 7,1, -1, 4, 5, 6, 8, -7 };
                GeoUtility.DS_Algo<int>.st_1(data).ForEach(a => { Console.Write(a.ToString() + ","); });
                Console.WriteLine("");
                GeoUtility.DS_Algo<int>.st_2(data).ForEach(a => { Console.Write(a.ToString() + ","); });
                Console.WriteLine("");
                GeoUtility.DS_Algo<int>.st_3(data).ForEach(a => { Console.Write(a.ToString() + ","); });
                Console.WriteLine("");
                GeoUtility.DS_Algo<int>.st_4(data).ForEach(a => { Console.Write(a.ToString() + ","); });
                Console.WriteLine("");
                GeoUtility.DS_Algo<int>.st_5(data).ForEach(a => { Console.Write(a.ToString() + ","); });
                Debugger.Break();
            }
        }

        public MainWindow()
        {
            InitializeComponent();
            UnLock();
            eyeshot.PreviewMouseDown+=eyeshot_PreviewMouseDown;
            eyeshot.InitializeScene += eyeshot_InitializeScene;
            InitViewport();
            InitMaterial();
            InitLayer();
            InitEntity();
        }

        void eyeshot_InitializeScene(object sender, EventArgs e)
        {
            eyeshot.Camera.ProjectionMode = projectionType.Orthographic;

            // CreateBox();
            // TestSurf();
            // TestCoplanar();
#if DEBUG

            //TestIO();
            //TestSlice();
             //TestHE();
             TestMC();
            //TestDSAlgo();
#else
            CheckInterference();
#endif
        }

        private void TestCoplanar()
        {
            //Mesh m1 = ImportModel("c:/users/peijingdong/desktop/mm1.stl");
            //Mesh m2 = ImportModel("c:/users/peijingdong/desktop/mm2.stl");

            //Plane p1 = Utility.FitPlane(m1.Vertices);
            //Plane p2 = Utility.FitPlane(m2.Vertices);

            //double d =Math.Abs( m1.Vertices[0].DistanceTo(p2));

            //Segment3D seg;
            //var res= Plane.Intersection(p1, p2, out seg);
            
            //Debugger.Break();
        }

        private void UnLock()
        {
            FileStream fs = new FileStream("../../key.lic",FileMode.Open);
            StreamReader sr = new StreamReader(fs);
            var key=sr.ReadLine();
            eyeshot.Unlock(key);
            sr.Close();
            fs.Close();
        }

        private void InitViewport()
        {
            eyeshot.Rendered.ShowEdges = false;
            eyeshot.Rendered.ShowInternalWires = false;
            eyeshot.Rendered.PlanarReflections = false;
            eyeshot.GetGrid().Visible = false;
            eyeshot.Backface.ColorMethod = backfaceColorMethodType.SingleColor;
            eyeshot.Backface.Color = Color.Yellow;
        }

        private List<Triangle> ImportTriangles(string path)
        {
            FileStream fs = new FileStream(path, FileMode.Open);
            StreamReader sr = new StreamReader(fs);
            string line;
            List<Triangle> tris = new List<Triangle>();
            while ((line = sr.ReadLine()) != null)
            {
                var cur = line.Split(new char[] { ' ', ',', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (cur.Length != 9)
                    continue;
                Point3D[] pts = new Point3D[3];
                for (int i = 0; i < 3; i++)
                {
                    pts[i] = new Point3D(double.Parse(cur[i * 3 + 0]), double.Parse(cur[i * 3 + 1]), double.Parse(cur[i * 3 + 2]));
                }
                tris.Add(new Triangle(pts[0], pts[1], pts[2]));
            }
            sr.Close();
            fs.Close();
            return tris;
        }

        private void ShowTriangles(List<Triangle> tris,System.Drawing.Color color)
        {
            eyeshot.Entities.AddRange(tris, 0, color);
            eyeshot.Invalidate();
        }
        
        private void InitMaterial()
        {
            Material mat = new Material(Color.Black, Color.DeepSkyBlue, Color.Black, 0.0f,0.0f);
            eyeshot.Materials.Add(m_DefaultMaterial, mat);
           
        }

        private void InitLayer()
        {
            m_LayerList = new Dictionary<string,int>();
            m_LayerList.Add(m_MeshLayer, eyeshot.Layers.Add(m_MeshLayer));
            m_LayerList.Add(m_PointCloudLayer, eyeshot.Layers.Add(m_PointCloudLayer));
        }

        private void InitEntity()
        {
            m_MeshList = new List<Mesh>();
            m_PointList = new List<Point3D>();
        }

        private List<Point3D> ImportPointCloud(string path,out Location loc)
        {
            loc = new Location(new Point3D(),new Quaternion(1,1,1));
            FileStream fs = new FileStream(path, FileMode.Open);
            StreamReader sr = new StreamReader(fs);
            string line;
            List<Point3D> pts = new List<Point3D>();
            string pattern = @"{-?[0-9].*[0-9]}";
            while ((line=sr.ReadLine())!=null)
            {
                var res = Regex.Match(line, pattern, RegexOptions.IgnoreCase);
                if(res.Success)
                {
                    var buf = res.Value.Substring(1, res.Value.Length - 2);
                    var cur = buf.Split(new char[] { ' ', ',', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (cur.Length == 7)
                    {
                        sr.Close();
                        fs.Close();
                        loc= new Location(new Point3D(double.Parse(cur[4]), double.Parse(cur[5]), double.Parse(cur[6])),
                            new Quaternion(double.Parse(cur[0]), double.Parse(cur[1]), double.Parse(cur[2]), double.Parse(cur[3])));
                        return null;
                    }
                    else
                    {
                        pts.Add(new Point3D(double.Parse(cur[0]), double.Parse(cur[1]), double.Parse(cur[2])));
                    }
                }
                else
                {
                    var cur = line.Split(new char[]{' ', ',', '\t'},StringSplitOptions.RemoveEmptyEntries);
                    if (cur.Length == 7)
                    {
                        sr.Close();
                        fs.Close();
                        loc = new Location(new Point3D(double.Parse(cur[4]), double.Parse(cur[5]), double.Parse(cur[6])),
                            new Quaternion(double.Parse(cur[0]), double.Parse(cur[1]), double.Parse(cur[2]), double.Parse(cur[3])));
                        return null;
                    }
                    else
                    {
                        pts.Add(new Point3D(double.Parse(cur[0]), double.Parse(cur[1]), double.Parse(cur[2])));
                    }
                }
               
            }
            sr.Close();
            fs.Close();
            return pts;
        }

        private Mesh ImportModel(string path)
        {
            ReadSTL stl = new ReadSTL(path, false,Mesh.natureType.Plain);
            eyeshot.DoWork(stl);
           // return (Mesh)stl.Entities[0];
            return (stl.Entities[0] as IFace).GetPolygonMeshes()[0];
        }

        private void ShowPoints(IList<Point3D> ps, Color color, bool showCurve=false,float size = 5)
        {
            try
            {
                eyeshot.Entities.Add(new PointCloud(ps, size), m_LayerList[m_PointCloudLayer], color);
                eyeshot.Entities.Add(new Joint(ps[0], 0.25, 1), m_LayerList[m_PointCloudLayer], Color.Red);
                eyeshot.Entities.Add(new Joint(ps[ps.Count-1], 0.25, 1), m_LayerList[m_PointCloudLayer], Color.Yellow);
                if (showCurve)
                {
                    LinearPath lp = new LinearPath(ps);
                    eyeshot.Entities.Add(lp, 0, Color.Black);
                }
                eyeshot.Invalidate();
            }
            catch (Exception)
            {

            }
        }

        private void ShowPoint(Point3D p, Color color, double radius = 0.5)
        {
            Joint j = new Joint(p, radius, 1);
            eyeshot.Entities.Add(j, m_LayerList[m_PointCloudLayer], color);
        }

        private void ShowMeshes(params Mesh[] meshes)
        {
            foreach (var item in meshes)
            {
                Color c = RandomColor();
                eyeshot.Entities.Add(item, m_LayerList[m_MeshLayer],c);
            }
            eyeshot.Invalidate();
        }

        private void ImportMesh_Click(object sender, RoutedEventArgs e)
        {
            ImportMesh.IsEnabled = false;
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Title = "选择stl文件";
            openFileDialog.Filter = "stl文件|*.stl|所有文件|*.*";
            openFileDialog.Multiselect = true;
            //openFileDialog.InitialDirectory = System.Windows.Forms.Application.StartupPath + "\\model";
            openFileDialog.RestoreDirectory = true;
            openFileDialog.DefaultExt = "stl";
            DialogResult result = openFileDialog.ShowDialog();
            if (result == System.Windows.Forms.DialogResult.Cancel)
            {
                ImportMesh.IsEnabled = true;
                return;
            }
           string[] fileNameList = openFileDialog.FileNames;
           Task ioTask = new System.Threading.Tasks.Task((Action)delegate
           {
               foreach (var item in fileNameList)
               {
                   Mesh mesh = ImportModel(item);
                   // mesh.ApplyMaterial(m_DefaultMaterial, textureMappingType.Cubic, 1, 1);         
                   mesh.UpdateNormals();
                   m_MeshList.Add(mesh);
                   Dispatcher.BeginInvoke((Action)delegate{ ShowMeshes(mesh);});
               }
               Dispatcher.BeginInvoke((Action)delegate{ ImportMesh.IsEnabled = true;});
           });
           ioTask.Start();
           //Dispatcher.BeginInvoke((Action)delegate
           //{
           //    foreach (var item in fileNameList)
           //    {
           //        Mesh mesh = ImportModel(item);
           //       // mesh.ApplyMaterial(m_DefaultMaterial, textureMappingType.Cubic, 1, 1);         
           //        mesh.UpdateNormals();
           //        m_MeshList.Add(mesh);

           //        ShowMeshes(mesh);
           //    }
           //    ImportMesh.IsEnabled = true;
           //});
            
        }

        private void ImportPoints_Click(object sender, RoutedEventArgs e)
        {
            ImportPoints.IsEnabled = false;
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Title = "选择txt文件";
            openFileDialog.Filter = "txt文件|*.txt|所有文件|*.*";
            //openFileDialog.InitialDirectory = System.Windows.Forms.Application.StartupPath + "\\model";
            openFileDialog.RestoreDirectory = true;
            openFileDialog.DefaultExt = "txt";
            DialogResult result = openFileDialog.ShowDialog();
            if (result == System.Windows.Forms.DialogResult.Cancel)
            {
                ImportPoints.IsEnabled = true;
                return;
            }
            string fileName = openFileDialog.FileName;
            Dispatcher.BeginInvoke((Action)delegate
            {
               // eyeshot.Entities.Clear();
                Location loc;
                List<Point3D> pointList = ImportPointCloud(fileName,out loc);
                if (pointList == null)
                {
                    loc.Show(eyeshot, 0);
                }
                else
                {
                    m_PointList.Clear();
                    m_PointList.AddRange(pointList);

                    ShowPoints(m_PointList, RandomColor(),true);
                    //CheckInterference();
                    //eyeshot.SetView(viewType.Top);
                    //eyeshot.ZoomFit();
                    //eyeshot.Invalidate();
                }
                ImportPoints.IsEnabled = true;
            });
        }

        private void ClearMesh_Click(object sender, RoutedEventArgs e)
        {
            eyeshot.Layers[m_LayerList[m_MeshLayer]].Visible = false;
            eyeshot.Invalidate();

          //  Move(moveType.xp);
        }

        private void ClearPoints_Click(object sender, RoutedEventArgs e)
        {
            eyeshot.Layers[m_LayerList[m_PointCloudLayer]].Visible = false;
            eyeshot.Invalidate();
           // Move(moveType.xn);

        }

        private void ClearAll_Click(object sender, RoutedEventArgs e)
        {
            if (true||
                System.Windows.MessageBox.Show("clear all?", "warnings",
                System.Windows.MessageBoxButton.YesNo) == System.Windows.MessageBoxResult.Yes)
            {
                eyeshot.Layers[m_LayerList[m_PointCloudLayer]].Visible = true;
                eyeshot.Layers[m_LayerList[m_MeshLayer]].Visible = true;

                eyeshot.Entities.Clear();
                eyeshot.Invalidate();
            }
            //Move(moveType.yp);

        }

        private void ShowAll_Click(object sender, RoutedEventArgs e)
        {
            eyeshot.Layers[m_LayerList[m_PointCloudLayer]].Visible = true;
            eyeshot.Layers[m_LayerList[m_MeshLayer]].Visible = true;
            eyeshot.Invalidate();
           // Move(moveType.yn);

        }

        private void DisplayMode_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            eyeshot.DisplayMode=(displayType)DisplayMode.SelectedIndex;
            eyeshot.Invalidate();
        }

        private void MeasureDist_Click(object sender, RoutedEventArgs e)
        {
            //CheckInterference(true);
#if DEBUG
            eyeshot.Entities.Clear();
            RunIO_Tree();
            eyeshot.Entities.Add(_wnTree._mesh, 0, Color.FromArgb(200, Color.Black));
            eyeshot.Invalidate();
#endif
        }

        void eyeshot_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            // gets the entity index
            int entityIndex = eyeshot.GetEntityUnderMouseCursor(RenderContextUtility.ConvertPoint(e.GetPosition(eyeshot)));

            // if we found an entity and the left mouse button is down            
            if (entityIndex != -1 && e.LeftButton == MouseButtonState.Pressed)
            {
                try
                {
                    // gets the entity reference
                    Entity entity = eyeshot.Entities[entityIndex] as Entity;
                    if(entity is Joint)
                    {
                        var joint = entity as Joint;
                        BaseIO(joint.Position);
                    }

                }
                catch
                {
                    return;
                }
                //Debugger.Break();
            }
        }

        private void eyeshot_Drop(object sender, System.Windows.DragEventArgs e)
        {
            string[] filenames = (string[])(e.Data.GetData(System.Windows.DataFormats.FileDrop));

            try
            {
                foreach (var item in filenames)
                {
                    var mesh = ImportModel(item);
                    eyeshot.Entities.Add(mesh, 0, RandomColor());
                }
                eyeshot.Invalidate();
            }
            catch (Exception)
            {
                System.Windows.MessageBox.Show("unsupport format!\n");
            }

        }
    
    }
}
