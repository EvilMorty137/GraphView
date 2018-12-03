using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using devDept.Eyeshot.Entities;
using devDept.Geometry;
using devDept.Eyeshot.Triangulation;

using MathNet;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using devDept.Eyeshot;

namespace GeoUtility
{
    public struct Property
    {
        public bool visited;
        public double value;
        public int prev;
        //public int next;
    }

    public class Edge:IndexLine
    {
        //public int adjFace;
        // public Property prop;
        public HalfEdge hedge;

        public Edge() : base() { }
        public Edge(int v1, int v2):base(v1,v2){ }
        public Edge(IndexLine another) : base(another) { }
        public Edge(Edge another) { V1 = another.V1; V2 = another.V2; }
        public Edge(HalfEdge another) { V1 = another.prev.end_vertex_id;V2 = another.end_vertex_id; }
}

    public class HalfEdge
    {
        public HalfEdge next;
        public HalfEdge prev;
        public HalfEdge pair;
        public Face left_face;
        public Vertex end_vertex;
        public Edge edge;
        public Property prop;
        public int left_face_id;
        public int end_vertex_id;

        public HalfEdge()
        {

        }
        public HalfEdge(int f_id, int ev_id)
        {
            left_face_id = f_id;
            end_vertex_id = ev_id;
        }
        public void Set(int f_id, int ev_id)
        {
            left_face_id = f_id;
            end_vertex_id = ev_id;
        }
        public bool IsB { get { return left_face == null; } }
    }

    public class Face : IndexTriangle
    {
        public HalfEdge hedge;
        //public Property prop;

        public Face() : base() { }
        public Face(int v1, int v2, int v3) : base(v1, v2, v3) { }
        public Face(IndexTriangle another) : base(another) { }
        public Face(Face another) { V1 = another.V1; V2 = another.V2; V3 = another.V3; }
    }

    public class Vertex : Point3D,IComparable
    {
        public HalfEdge out_hedge;
        public Property prop;

        public Vertex() : base() { }
        public Vertex(double[] coords) : base(coords) { }
        public Vertex(double x, double y) : base(x, y) { }
        public Vertex(double x, double y, double z) : base(x, y, z) { }
        public Vertex(Point3D another) : base(another) { }
        public Vertex(Vertex another) { X = another.X; Y = another.Y; Z = another.Z; }
        public int CompareTo(object obj){ return prop.value.CompareTo((obj as Vertex).prop.value)*-1;}
    }



    public abstract class HalfMeshBase:Mesh
    {
        protected Dictionary<int, int> m_old_new_vid;
        protected Dictionary<Vertex, int> m_vertex_id;
        protected Dictionary<Edge, HalfEdge> m_edge_hedge;

        public List<Vertex> vList;
        public List<Face> fList;
        public List<HalfEdge> heList;

        public HalfMeshBase()
        {
            Init();
            Console.WriteLine(" HalfMeshBase default construct!");
        }
        public HalfMeshBase(Mesh mesh)
        {
            Init();
            Dump(mesh);
            Console.WriteLine(" HalfMeshBase param construct!");
        }

        public int VertexIndex(Vertex v) { return m_vertex_id[v]; }
        public Vertex FindVertex(Point3D p)
        {
            var v = new Vertex(p);
            if (m_vertex_id.ContainsKey(v))
                return vList[m_vertex_id[v]];
            return null;
        }

        public abstract List<Vertex> VV(Vertex v);
        public abstract List<Face> VF(Vertex v);
        public abstract List<HalfEdge> VEO(Vertex v);
        public abstract List<HalfEdge> VEI(Vertex v);
        public abstract List<Face> FF(Face f);
        public abstract List<List<HalfEdge>> FindHoles();
        public abstract void AddFace(IndexTriangle t);
        public abstract bool IsBoundary(HalfEdge he);

        public virtual void AddVertex(Point3D p)
        {
            Vertex v = new Vertex(p);
            if (!m_vertex_id.ContainsKey(v))
            {
                m_vertex_id.Add(v, m_vertex_id.Count);
                vList.Add(v);
            }
            m_old_new_vid.Add(m_old_new_vid.Count, vList.Count - 1);
        }
        public virtual double GD(Vertex src, Vertex dest, out List<Vertex> path)
        {
            path = new List<Vertex>();

            SortedDictionary<double, Vertex> gDistBuf = new SortedDictionary<double, Vertex>();

            vList.ForEach(a => { a.prop.visited = false; a.prop.value = double.MaxValue; });
            src.prop.value = 0;

            gDistBuf.Add(src.prop.value, src);
            while (true)
            {
                var cur = gDistBuf.ElementAt(0);
                gDistBuf.Remove(cur.Key);

                if (cur.Value.prop.visited)
                    continue;
                cur.Value.prop.visited = true;

                foreach (var vv in VV(cur.Value))
                {
                    if (vv.prop.visited)
                        continue;

                    double d = cur.Key + vv.DistanceTo(cur.Value);
                    if (d < vv.prop.value)
                    {
                        vv.prop.prev = m_vertex_id[cur.Value];

                        vv.prop.value = d;
                        gDistBuf.Add(d, vv);
                    }
                }

                if (dest.prop.visited)
                    break;
            }

            {
                //get path
                var cur = dest;
                while (cur != src)
                {
                    path.Add(cur);
                    cur = vList[cur.prop.prev];
                }
                path.Add(src);
            }

            return dest.prop.value;
        }
        public virtual double GD_OLD(Vertex src, Vertex dest, out List<Vertex> path)
        {
            path = new List<Vertex>();
            MaxHeap<Vertex> geoDistMinHeap = new MaxHeap<Vertex>();

            vList.ForEach(a => { a.prop.visited = false; a.prop.value = double.MaxValue; a.prop.prev = -1; });
            src.prop.value = 0;

            geoDistMinHeap.Add(src);
            while (true)
            {
                var cur = geoDistMinHeap.Top();
                geoDistMinHeap.Pop();

                if (cur.prop.visited)
                    continue;
                cur.prop.visited = true;

                foreach (var vv in VV(cur))
                {
                    if (vv.prop.visited)
                        continue;
                    double d = cur.prop.value + vv.DistanceTo(cur);
                    if (d < vv.prop.value)
                    {
                        if (!m_vertex_id.ContainsKey(cur))
                        {
                            Console.WriteLine("A Error on Topo!");
                            return -1;
                        }
                        vv.prop.prev = m_vertex_id[cur];

                        vv.prop.value = d;
                        geoDistMinHeap.Add(vv);
                    }
                }

                if (dest.prop.visited)
                    break;
            }

            {
                //get path
                var cur = dest;
                while (cur != src)
                {
                    path.Add(cur);
                    cur = vList[cur.prop.prev];
                }
                path.Add(src);
            }

            return dest.prop.value;
        }

        protected virtual void Init()
        {
            m_old_new_vid = new Dictionary<int, int>();
            m_vertex_id = new Dictionary<Vertex, int>();
            m_edge_hedge = new Dictionary<Edge, HalfEdge>();

            vList = new List<Vertex>();
            fList = new List<Face>();
            heList = new List<HalfEdge>();
            Console.WriteLine(" HalfMeshBase init!");
        }
        protected virtual void Dump(Mesh mesh)
        {
            this.Vertices = mesh.Vertices;
            this.Triangles = mesh.Triangles;
        }
    }

    public class HEMesh:HalfMeshBase
    {
        public List<Edge> eList;

        public HEMesh()
        {
        }

        public HEMesh(Mesh mesh):base(mesh)
        {
            foreach (var item in mesh.Vertices)
                AddVertex(item);
            foreach (var item in mesh.Edges)
                AddEdge(item);
            foreach (var item in mesh.Triangles)
                AddFace(item);
            //System.Diagnostics.Debugger.Break();
            Console.WriteLine("HEMesh constructed done!");
        }

        private HalfEdge FindFreeIncident(Vertex v)
        {
            return  v.out_hedge.prev;
        }

        private HalfEdge FindExistHalfEdge(Vertex from ,Vertex to)
        {
            if(from.out_hedge!=null&&from.out_hedge.end_vertex==to)
                return from.out_hedge;
            return null;
        }

        private Edge AddEdge(IndexLine e)
        {
            return AddEdge(vList[m_old_new_vid[e.V1]], vList[m_old_new_vid[e.V2]]);
        }

        private Edge AddEdge(Vertex from,Vertex to)
        {
            int from_id = m_vertex_id[from];
            int to_id = m_vertex_id[to];

            if(from==to)
            {
                Console.WriteLine("duplicate vertex on edge!");
                return new Edge();
            }

            HalfEdge exist_he = FindExistHalfEdge(from, to);
            if(exist_he!=null)
            {
                Console.WriteLine("edge has been added!");
                return exist_he.edge;
            }

            Edge e = new Edge(from_id, to_id);
            eList.Add(e);

            HalfEdge he_from_to = new HalfEdge();
            HalfEdge he_to_from = new HalfEdge();
            heList.Add(he_from_to);
            heList.Add(he_to_from);
            e.hedge = he_from_to;

            {
                he_from_to.end_vertex = to;
                he_from_to.edge = e;
                he_from_to.next = he_to_from;
                he_from_to.prev = he_to_from;
                he_from_to.pair = he_to_from;
            }

            {
                he_to_from.end_vertex = from;
                he_to_from.edge = e;
                he_to_from.next = he_from_to;
                he_to_from.prev = he_from_to;
                he_to_from.pair = he_from_to;
            }

            {
                HalfEdge he_from_in = FindFreeIncident(from);
                HalfEdge he_form_out = he_from_in.next;

                he_from_to.prev = he_from_in;
                he_from_in.next = he_from_to;

                he_to_from.next = he_form_out;
                he_form_out.prev = he_to_from;
            }

            {
                HalfEdge he_to_in = FindFreeIncident(to);
                HalfEdge he_to_out = he_to_in.next;

                he_to_from.prev = he_to_in;
                he_to_in.next = he_to_from;

                he_from_to.next = he_to_out;
                he_to_out.prev = he_from_to;
            }

            return e;
        }

        public override void AddFace(IndexTriangle t)
        {
            Face f = new Face(t);
            {
                f.V1 = m_old_new_vid[f.V1];
                f.V2 = m_old_new_vid[f.V2];
                f.V2 = m_old_new_vid[f.V2];
            }
            
            fList.Add(f);
            int fId = fList.Count - 1;
            int[] vs = new int[3] { f.V1, f.V2, f.V3 };

            HalfEdge[] hes = new HalfEdge[3];
            HalfEdge[] bhes = new HalfEdge[3];
            bool[] exist_hes = new bool[3];

            for (int i = 0; i < 3; i++)
            {
                Edge edge_ccw = new Edge(vs[i], vs[(i + 1) % 3]);
                Edge edge_cw = new Edge(vs[(i + 1) % 3], vs[i]);

                //edge exist
                if (m_edge_hedge.ContainsKey(edge_cw))
                {
                    //get edge->he
                    hes[i] = edge_cw.hedge.pair;
                    bhes[i] = edge_cw.hedge;
                    exist_hes[i] = true;
                }
                else
                {
                    //allocate he
                    hes[i] = new HalfEdge();
                    bhes[i] = new HalfEdge();
                    exist_hes[i] = false;

                    //set e<->he
                    hes[i].edge = edge_ccw;
                    edge_ccw.hedge = hes[i];
                    bhes[i].edge = edge_cw;
                    edge_cw.hedge = bhes[i];

                    m_edge_hedge.Add(edge_ccw, null);
                    m_edge_hedge.Add(edge_cw, null);

                }
            }

            for (int i = 0; i < 3; i++)
            {
                if (!exist_hes[i])
                {
                    //set he prev&next
                    hes[i].next = hes[(i + 1) % 3];
                    hes[i].prev = hes[(i + 2) % 3];
                    bhes[i].next = bhes[(i + 2) % 3];
                    bhes[i].prev = bhes[(i + 1) % 3];

                    //set he pair
                    hes[i].pair = bhes[i];
                    bhes[i].pair = hes[i];

                    //set he end vertex
                    hes[i].end_vertex = vList[vs[(i + 1) % 3]];
                    bhes[i].end_vertex = vList[vs[i]];

                }

                //set v->he??
                vList[vs[i]].out_hedge = hes[i];

                //set he<->face
                hes[i].left_face = f;
                f.hedge = hes[i];

            }

            for (int i = 0; i < 3; i++)
            {
//                 Vertex v = vList[vs[i]];
//                 if (m_vertex_id.ContainsKey(v))
//                 {
// 
//                 }
            }

        }

        public void AddFace(IList<HalfEdge> loop)
        {
            if(loop.Count!=3)
            {
                Console.WriteLine("loop is not triangle!");
                return;
            }

           


        }

        protected override void Init()
        {
            base.Init();
            eList = new List<Edge>();
            Console.WriteLine(" HEMesh init!");
        }

        public override List<Vertex> VV(Vertex v)
        {
            List<Vertex> res = new List<Vertex>();

            var begin = v.out_hedge;
            var cur = begin;
            do
            {
                res.Add(cur.end_vertex);
                cur = cur.pair.next;
            } while (cur!=begin);

            return res;
        }

        public override List<Face> VF(Vertex v)
        {
            List<Face> res = new List<Face>();

            var begin = v.out_hedge;
            var cur = begin;
            do
            {
                if(!cur.IsB)
                    res.Add(cur.left_face);
                cur = cur.pair.next;
            } while (cur != begin);

            return res;
        }

        public override List<HalfEdge> VEO(Vertex v)
        {
            List<HalfEdge> res = new List<HalfEdge>();

            var begin = v.out_hedge;
            var cur = begin;
            do
            {
                res.Add(cur);
                cur = cur.pair.next;
            } while (cur != begin);

            return res;
        }

        public override List<HalfEdge> VEI(Vertex v)
        {
            List<HalfEdge> res = new List<HalfEdge>();

            var begin = v.out_hedge.pair;
            var cur = begin;
            do
            {
                res.Add(cur);
                cur = cur.next.pair;
            } while (cur != begin);

            return res;
        }

        public override List<Face> FF(Face f)
        {
            List<Face> res = new List<Face>();

            var begin = f.hedge;
            var cur = begin;
            do
            {
                if (!cur.pair.IsB)
                    res.Add(cur.pair.left_face);
                cur = cur.next;
            } while (cur != begin);

            return res;
        }

        public override List<List<HalfEdge>> FindHoles()
        {
            List<List<HalfEdge>> holes = new List<List<HalfEdge>>();

            heList.ForEach(a => a.prop.visited = false);

            foreach (var hedge in heList)
            {
                if (hedge.prop.visited)
                    continue;

                if(hedge.IsB)
                {
                    List<HalfEdge> hole = new List<HalfEdge>();

                    var begin = hedge;
                    var cur = begin;
                    do
                    {
                        cur.prop.visited = true;
                        hole.Add(cur);
                        cur = cur.next;
                    } while (cur!=begin);

                    holes.Add(hole);
                }
            }

            return holes;
        }

        public override bool IsBoundary(HalfEdge he)
        {
            if (he == null)
                return true;
            return he.pair == null;
        }
    }

    public class Polyhedron : HalfMeshBase
    {
        public Polyhedron()
        {
        }

        public Polyhedron(Mesh mesh):base(mesh)
        {
            foreach (var item in mesh.Vertices)
                AddVertex(item);
            foreach (var item in mesh.Triangles)
                AddFace(item);
            //System.Diagnostics.Debugger.Break();
            Console.WriteLine("polyhedron constructed done!");
        }

        public override List<Vertex> VV(Vertex v)
        {
            List<Vertex> nbhd = new List<Vertex>();
            HalfEdge he = v.out_hedge;
            do
            {
                nbhd.Add(vList[he.end_vertex_id]);
                if (he.prev.pair == null)//boundary 
                {
                    nbhd.Add(vList[he.next.end_vertex_id]);
                    if (v.out_hedge.pair != null)
                        he = v.out_hedge.pair.next;
                    else
                        break;
                }
                else
                    he = he.prev.pair;
            } while (he != v.out_hedge && he != null);
            return nbhd;
        }

        public override List<Face> VF(Vertex v)
        {
            List<Face> nbhd = new List<Face>();
            HalfEdge he = v.out_hedge;
            do
            {
                nbhd.Add(fList[he.left_face_id]);
                if (he.prev.pair == null)//boundary 
                {
                    if (v.out_hedge.pair != null)
                    {
                        he = v.out_hedge.pair.next;
                    }
                }
                else
                {
                    he = he.prev.pair;
                }
            } while (he != v.out_hedge && he != null);
            return nbhd;
        }

        public override List<HalfEdge> VEO(Vertex v)
        {
            List<HalfEdge> nbhd = new List<HalfEdge>();
            HalfEdge he = v.out_hedge;
            do
            {
                nbhd.Add(he);
                if (he.prev.pair == null)//boundary 
                {
                    if (v.out_hedge.pair != null)
                        he = v.out_hedge.pair.next;
                }
                else
                    he = he.prev.pair;
            } while (he != v.out_hedge && he != null);
            return nbhd;
        }

        public override List<HalfEdge> VEI(Vertex v)
        {
            List<HalfEdge> nbhd = new List<HalfEdge>();
            HalfEdge he = v.out_hedge.prev;
            do
            {
                nbhd.Add(he);
                if (he.next.pair == null)//boundary 
                    break;
                else
                    he = he.next.pair;
            } while (he != v.out_hedge.prev && he != null);
            return nbhd;
        }

        public override List<Face> FF(Face f)
        {
            List<Face> nbhd = new List<Face>();
            HalfEdge he= f.hedge;
            do
            {
                if (he.pair != null)
                    nbhd.Add(fList[he.pair.left_face_id]);
                he = he.next;
            } while (he != f.hedge);
            return nbhd;
        }

        public override List<List<HalfEdge>> FindHoles()
        {
            List<List<HalfEdge>> holes = new List<List<HalfEdge>>();
            heList.ForEach(he => he.prop.visited=false);

            foreach (var item in heList)
            {
                if (item.prop.visited)
                    continue;
                if (IsBoundary(item))
                {
                    List<HalfEdge> hole = new List<HalfEdge>();
                    HalfEdge he = item;
                    do
                    {
                        he.prop.visited = true;
                        hole.Add(he);
                        do
                        {
                            he = he.next;
                            if (IsBoundary(he))
                                break;
                            he = he.pair;
                        } while (true);
                    } while (he != item);
                    holes.Add(hole);
                }
            }
            return holes;
        }    

        public override void AddFace(IndexTriangle t)
        {
            Face f = new Face(t);
            f.V1 = m_old_new_vid[f.V1];f.V2 = m_old_new_vid[f.V2];f.V2 = m_old_new_vid[f.V2];
            fList.Add(f);
            int fId = fList.Count - 1;
            
            HalfEdge[] hes = new HalfEdge[3];
            for (int i = 0; i < 3; i++)
            {
                hes[i] = new HalfEdge();
                heList.Add(hes[i]);
            }

            #region ref he
            //HalfEdge he31 = hes[0];
            //HalfEdge he12 = hes[1];
            //HalfEdge he23 = hes[2];
            #endregion

            int[] vs = new int[3] { f.V1, f.V2, f.V3 };
            for (int i = 0; i < 3; i++)
            {
                //create he  
                hes[i].Set(fId, vs[i]);

                //set he->prev&next
                hes[i].next = hes[(i + 1) % 3];
                hes[i].prev = hes[(i + 2) % 3];

                //set v->he
                vList[vs[i]].out_hedge = hes[(i + 1) % 3];

                //set f->he
                if (i == 0)
                    f.hedge = hes[i];

                //set he->twin
                var he = hes[(i + 1) % 3];
                Edge eCCW = new Edge(vs[i], vs[(i + 1) % 3]);//CCW
                Edge eCW = new Edge(vs[(i + 1) % 3], vs[i]);//CW
                if (!m_edge_hedge.ContainsKey(eCCW))
                    m_edge_hedge.Add(eCCW, he);
                else
                {
                    Console.WriteLine("warning: non-manifold edge!");
                   // System.Diagnostics.Debugger.Break();
                }
                if (m_edge_hedge.ContainsKey(eCW))
                {
                    he.pair = m_edge_hedge[eCW];
                    m_edge_hedge[eCW].pair = he;
                }

            }

            #region ref he
            //create he          
            //he31.Set(fId, vs[0]);
            //he12.Set(fId, vs[(0 + 1) % 3]);
            //he23.Set(fId, vs[(0 + 2) % 3]);

            //set he->prev&next
            //he31.next = he12; he31.prev = he23;
            //he12.next = he23; he12.prev = he31;
            //he23.next = he31; he23.prev = he12;

            //set v->he
            //vList[f.V1].hEdgeOut = he12;
            //vList[f.V2].hEdgeOut = he23;
            //vList[f.V3].hEdgeOut = he31;

            //set f->he
            //f.hEdge = he31;
            #endregion

        }

        public override bool IsBoundary(HalfEdge he)
        {
            if (he == null)
                return true;
            return he.pair == null;
        }

    }


    

    class BiNode<T>
    {
        public T value;
        public BiNode<T> left;
        public BiNode<T> right;


        public int depth;
        public bool IsLeaf { get { return left == null && right == null; } }

        public BiNode()
        {
            left = null;
            right = null;
        }

        public BiNode(T v,int d)
        {
            depth = d;
            value = v;
            left = null;
            right = null;
        }

        public BiNode(T v)
        {
            value = v;
            left = null;
            right = null;
        }

        public BiNode(T v, BiNode<T> l, BiNode<T> r)
        {
            value = v;
            left = l;
            right = r;
        }

        public bool IsParent(BiNode<T> node)
        {
            return node != null && (left == node || right == node);
        }

    }

    class MaxHeap<T> where T : IComparable
    {
        protected List<T> m_Data = new List<T>();

        public List<T> Data { get { return m_Data; } }

        public MaxHeap()
        {
        }

        public MaxHeap(IList<T> data, int method = 0)
        {
            if (method == 0)
            {
                m_Data.AddRange(data);

                int nbr = m_Data.Count;
                int end = (nbr - 1 - 1) / 2;
                for (int i = end; i >= 0; i--)
                {
                    Heapify(i, nbr);
                }
            }
            else
            {
                foreach (var item in data)
                {
                    Add(item);
                }
            }
        }

        public void Add(T val,bool resort=true)
        {
            m_Data.Add(val);

            if (resort)
            {
                int nbr = m_Data.Count;

                int cur = nbr - 1;
                while (cur > 0)
                {
                    int prev = (cur - 1) / 2;
                    if (val.CompareTo(m_Data[prev]) > 0)
                    {
                        m_Data[cur] = m_Data[prev];
                        cur = prev;
                    }
                    else
                    {
                        break;
                    }
                }
                m_Data[cur] = val;
            }

        }

        public T Top()
        {
            return m_Data[0];
        }

        public T Pop()
        {
            var res = m_Data[0];
            m_Data.RemoveAt(0);
            Heapify(0, m_Data.Count);
            return res;
        }

        public void Heapify(int b, int e)
        {
            if (b >= e || b < 0 || e > m_Data.Count)
                return;

            int nbr = e;
            var val = m_Data[b];
            int cur = b;

            while (cur < nbr)
            {
                int next = -1;
                if (2 * cur + 1 >= nbr)
                    break;
                else if (2 * cur + 2 >= nbr)
                    next = 2 * cur + 1;
                else
                    next = m_Data[2 * cur + 1].CompareTo(m_Data[2 * cur + 2]) > 0  ? 2 * cur + 1 : 2 * cur + 2;
                if (val.CompareTo(m_Data[next]) < 0 )
                {
                    m_Data[cur] = m_Data[next];
                    cur = next;
                }
                else
                {
                    break;
                }
            }
            m_Data[cur] = val;
        }

    }

    abstract class BiTreeBase<T>:ComponentBase
        where T : IComparable
    {
        public BiNode<T> root; 

        public BiTreeBase()
        {

        }

        public abstract void Add();

        public virtual List<T> PreTraverse()
        {
            Console.Write("\nPreTraverse:\t");

            if (root == null)
                return null;

            List<T> data = new List<T>();

            Stack<BiNode<T>> stack = new Stack<BiNode<T>>();
            stack.Push(root);

            while (stack.Count>0)
            {
                var cur = stack.Pop();
                data.Add(cur.value);

                if (cur.right != null)
                {
                    stack.Push(cur.right);
                }
                if (cur.left!=null)
                {
                    stack.Push(cur.left);
                }         
            }

            return data;
        }

        public virtual List<T> MidTraverse()
        {
            Console.Write("\nMidTraverse:\t");

            if (root == null)
                return null;

            List<T> data = new List<T>();

            Stack<BiNode<T>> stack = new Stack<BiNode<T>>();

            BiNode<T> cur = root;
            while (stack.Count > 0||cur!=null) 
            {
                if (cur != null)
                {
                    stack.Push(cur);
                    cur = cur.left;
                }
                else
                {
                    cur = stack.Pop();
                    data.Add(cur.value);
                    cur = cur.right;
                }
            } 

            return data;
        }

        public virtual List<T> PostTraverse()
        {
            Console.Write("\nPostTraverse:\t");

            if (root == null)
                return null;

            List<T> data = new List<T>();

            Stack<BiNode<T>> stack = new Stack<BiNode<T>>();
            stack.Push(root);

            BiNode<T> cur=null, prev = null;
            while (stack.Count > 0)
            {
                cur = stack.Peek();
                
                if(cur.IsLeaf||cur.IsParent(prev))
                {
                    prev = cur;
                    stack.Pop();
                    data.Add(cur.value);
                    continue;
                }

                if (cur.right != null)
                {
                    stack.Push(cur.right);
                }
                if (cur.left != null)
                {
                    stack.Push(cur.left);
                }

            }

            return data;
        }

        public virtual List<T> LevelTraverse()
        {
            Console.Write("\nLevelTraverse:\t");

            if (root == null)
                return null;

            List<T> data = new List<T>();

            Queue<BiNode<T>> queue = new Queue<BiNode<T>>();
            queue.Enqueue(root);

            while(queue.Count>0)
            {
                var cur = queue.Dequeue();
                data.Add(cur.value);

                if (cur.left != null)
                {
                    queue.Enqueue(cur.left);
                }
                if (cur.right != null)
                {
                    queue.Enqueue(cur.right);
                }
            }

            return data;
        }

        public virtual List<T> SelTraverse()
        {
            List<T> res = new List<T>();
            var stack = new Stack<BiNode<T>>();
            var cur = root;
            var prev = root;
            while (stack.Count>0||cur!=null)
            {
                if (cur !=null )
                {
                    stack.Push(cur);
                    cur = cur.left;
                }
                else
                {
                    cur = stack.Pop();
                    T max = prev.value;
                    foreach (var item in stack)
                    {
                        if(item.value.CompareTo(max)>=0 )
                            max = item.value;
                    }
                    if(cur.value.CompareTo(max)>=0)
                        res.Add(cur.value);
                    if (cur.right != null)
                        prev = cur;
                    cur = cur.right;
                }
            }

            return res;
        }

    }

    class BTree<T> : BiTreeBase<T>
        where T : IComparable
    {
        public override void Add()
        {
            throw new NotImplementedException();
        }
    }

    class Deformation
    {
        protected Mesh _mesh;
        public Deformation()
        {

        }
        public Deformation(Mesh mesh)
        {
            _mesh = mesh;
        }
        public virtual void Run(Dictionary<int,Point3D> contraints)
        {

    

        }

    }

    class MeshUtility
    {
        public static List<Triangle> TriangulateGreedy(IList<IndexLine> hole,Mesh mesh)
        {
            if (hole.Count == 0)
                return null;

            Point3D p = new Point3D();
            foreach (var item in hole)
                p += mesh.Vertices[item.V1];
            p /= hole.Count;
            //p = mesh.Vertices[hole[0].V1];

            List<Triangle> tris = new List<Triangle>();
            foreach (var item in hole)
                tris.Add(new Triangle(mesh.Vertices[item.V2], mesh.Vertices[item.V1], p));

            return tris;
        }
        public static List<Triangle> TriangulateGreedy(Dictionary<IndexLine, int> hole, Mesh mesh)
        {
            if (hole == null)
                return null;

            if (hole.Count == 0)
                return null;

            Point3D p = new Point3D();
            foreach (var item in hole)
                p += mesh.Vertices[item.Key.V1];
            p /= hole.Count;
            //p = mesh.Vertices[hole[0].V1];

            List<Triangle> tris = new List<Triangle>();
            foreach (var item in hole)
            {
                Triangle t = item.Value < 0 ?
                    new Triangle(mesh.Vertices[item.Key.V1], mesh.Vertices[item.Key.V2], p) :
                    new Triangle(mesh.Vertices[item.Key.V2], mesh.Vertices[item.Key.V1], p);
                tris.Add(t);
            }

            return tris;
        }
        public static List<IndexLine> FindHole(IList<IndexTriangle> faces)
        {
            Dictionary<IndexLine, int> mapOrientedEdges = new Dictionary<IndexLine, int>();
            foreach (var f in faces)
            {
                int[] vs = new int[3] { f.V1, f.V2, f.V3 };
                for (int i = 0; i < 3; i++)
                {
                    IndexLine eCCW = new IndexLine(vs[i], vs[(i + 1) % 3]);
                    IndexLine eCW = new IndexLine(vs[(i + 1) % 3], vs[i]);
                    if (mapOrientedEdges.ContainsKey(eCW))
                    {
                        ++mapOrientedEdges[eCW];
                    }
                    else
                    {
                        if(!mapOrientedEdges.ContainsKey(eCCW))
                            mapOrientedEdges.Add(eCCW, 0);
                    }
                }
            }

            List<IndexLine> holes = new List<IndexLine>();
            foreach (var item in mapOrientedEdges)
            {
                if (item.Value == 0)
                    holes.Add(item.Key);
            }
            return holes;

        }
        public static Dictionary<IndexLine,int> FindExteriorEdge(IList<IndexTriangle> faces)
        {
            if (faces == null)
                return null;

            Dictionary<IndexLine, int> mapOrientedEdges = new Dictionary<IndexLine, int>();
            foreach (var f in faces)
            {
                int[] vs = new int[3] { f.V1, f.V2, f.V3 };
                for (int i = 0; i < 3; i++)
                {
                    IndexLine e = new IndexLine(vs[i], vs[(i + 1) % 3]);
                    if (mapOrientedEdges.ContainsKey(e))
                    {
                        if (e.V1 < e.V2)
                            ++mapOrientedEdges[e];
                        else
                            --mapOrientedEdges[e];
                    }
                    else
                    {
                        mapOrientedEdges.Add(e, 0);
                    }
                }
            }

            Dictionary<IndexLine, int> holes = new Dictionary<IndexLine, int>();
            foreach (var item in mapOrientedEdges)
            {
                if (item.Value != 0)
                    holes.Add(item.Key, item.Value);
            }
            return holes;

        }
        public static double SolidAngle(IList<Point3D> ps, Point3D p)
        {
            if (ps.Count != 3)
                return 0;

            return SolidAngle(ps[0], ps[1], ps[2], p);
        }
        public static double SolidAngle(Point3D p1, Point3D p2, Point3D p3,Point3D p)
        {
            Vector3D a = new Vector3D(p, p1);
            Vector3D b = new Vector3D(p, p2);
            Vector3D c = new Vector3D(p, p3);

            double la = a.Length;
            double lb = b.Length;
            double lc = c.Length;

            double numerator = Vector3D.Dot(Vector3D.Cross(a, b), c);
            double denominator = la * lb * lc + 
                Vector3D.Dot(a, b) * lc + 
                Vector3D.Dot(b, c) * la + 
                Vector3D.Dot(c, a) * lb;

            if(Math.Abs(denominator) > 1e-6)
                return 2 * Math.Atan(numerator / denominator);

            return 0;
        }
        public static Point3D TriangleCenter(IndexTriangle face,Mesh mesh)
        {
            Point3D p = new Point3D();
            int[] vs = TriangleVertexIds(face);
            foreach (var item in vs)
            {
                p += mesh.Vertices[item];
            }
            p /= 3;
            return p;
        }
        public static int[] TriangleVertexIds(IndexTriangle face)
        {
          return  new int[3] { face.V1, face.V2, face.V3 };
        }
        public static double[] To3(Point3D p)
        {
            return new double[3] { p.X, p.Y, p.Z };
        }
    }

    abstract class ComponentBase
    {
        protected string _name;

        public string Name { get { return _name; } }

        public ComponentBase()
        {
            _name = "";
        }

        public ComponentBase(string name)
        {
            _name = name;
        }

        //public override int GetHashCode()
        //{
        //    return _name.GetHashCode();
        //}

        //public override bool Equals(object obj)
        //{
        //    return _name.Equals(obj);
        //}


    }

    abstract class ComponentManager
    {
        protected Dictionary<string, ComponentBase> _mapComponents = new Dictionary<string, ComponentBase>();
        public virtual void AddComponent(ComponentBase component)
        {
            if (component == null)
                return;
            if (!_mapComponents.ContainsKey(component.Name))
                _mapComponents.Add(component.Name,component);
        }
        public virtual void RemoveComponent(ComponentBase component)
        {
            if (component == null)
                return;
            if (_mapComponents.ContainsKey(component.Name))
                _mapComponents.Remove(component.Name);
        }
        public virtual void RemoveComponent(string compName)
        {
            if (_mapComponents.ContainsKey(compName))
                _mapComponents.Remove(compName);
        }
        public abstract ComponentBase GetComponent(string compName);

    }

    interface IRelativePositionDetectionBase
    {
        bool IsInside(Point3D p);
        void Init(Mesh mesh);
    }

    class OBBTree : ComponentBase
    {
        public OBBTree()
        {
            _name = "OBB";
        }
    }

    class KDTree : ComponentBase
    {
        public KDTree()
        {
            _name = "KD";
        }
    }

    class CMeshBase:ComponentManager
    {
        public Mesh _mesh;

        public CMeshBase()
        {

        }

        public CMeshBase(Mesh mesh)
        {
            _mesh = mesh;
        }

        public override ComponentBase GetComponent(string compName)
        {
            if (!_mapComponents.ContainsKey(compName))
            {
                switch (compName)
                {
                    case "RPD":
                        AddComponent(new RPDTree(_mesh));
                        break;
                    case "OBB":
                        AddComponent(new OBBTree());
                        break;
                    case "KD":
                        AddComponent(new KDTree());
                        break;
                    default:
                        break;
                }         
            }
            return _mapComponents[compName];
        }
    }

    class CMesh : CMeshBase
    {
        public CMesh():base()
        {

        }

        public CMesh(Mesh m) : base(m)
        {

        }
    }


    interface IVisible
    {
        void Show(ViewportLayout eyeshot, System.Drawing.Color color);
    }

    struct Box:IComparable,IVisible
    {
        public Point3D min;
        public Point3D max;

        public List<IndexTriangle> faces;
        public List<Triangle> sFaces;

        public double DX { get { return (max - min).X; } }
        public double DY { get { return (max - min).Y; } }
        public double DZ { get { return (max - min).Z; } }
        public Point3D Center { get { return (max + min) / 2; } }
        public Point3D Diag { get { return (max - min) ; } }

        public Box(Point3D min, Point3D max)
        {
            this.min = min;
            this.max = max;
            faces = new List<IndexTriangle>();
            sFaces = new List<Triangle>();
        }

        public Box(IList<Point3D> vertices)
        {
            if (vertices.Count == 0)
            {
                this.min = new Point3D();
                this.max = new Point3D();
                faces = new List<IndexTriangle>();
            }
            Point3D min = new Point3D(double.MaxValue, double.MaxValue, double.MaxValue);
            Point3D max = new Point3D(double.MinValue, double.MinValue, double.MinValue);
            foreach (var item in vertices)
            {
                min.X = Math.Min(item.X, min.X);
                min.Y = Math.Min(item.Y, min.Y);
                min.Z = Math.Min(item.Z, min.Z);

                max.X = Math.Max(item.X, max.X);
                max.Y = Math.Max(item.Y, max.Y);
                max.Z = Math.Max(item.Z, max.Z);
            }
            this.min = min;
            this.max = max;
            faces = new List<IndexTriangle>();
            sFaces = new List<Triangle>();
        }

        public Box(Mesh mesh)
        {
            this = new Box(mesh.Vertices);
            this.faces = mesh.Triangles.ToList();
        }

        public Box(IList<IndexTriangle> faces,Mesh mesh)
        {
            HashSet<int> faceVertexIds = new HashSet<int>();
            List<Point3D> vertices = new List<Point3D>();
            foreach (var item in faces)
            {
                var vs = MeshUtility.TriangleVertexIds(item);
                foreach (var v in vs)
                {
                    if (!faceVertexIds.Contains(v))
                    {
                        faceVertexIds.Add(v);
                        vertices.Add(mesh.Vertices[v]);
                    }
                }
            }
            this= new  Box(vertices);
            this.faces = faces.ToList();
        }

        public bool IsInside(Point3D p)
        {
            Vector3D c2p = new Vector3D(Center, p);
            var c2p3 = MeshUtility.To3(c2p);
            var diag = MeshUtility.To3(Diag);
            for (int i = 0; i < 3; i++)
            {
                if(c2p3[i]>diag[i])
                {
                    return false;
                }
            }
            return true;
        }

        public int CompareTo(object obj)
        {
            throw new NotImplementedException();
        }

        public void Show(ViewportLayout eyeshot, System.Drawing.Color color)
        {
            Mesh block = Mesh.CreateBox(DX, DY, DZ);
            Transformation trans = new Transformation();
            trans.Translation(new Vector3D(new Point3D(), min));
            block.TransformBy(trans);
            eyeshot.Entities.Add(block, 0, color);
            eyeshot.Invalidate();
        }
    }

    class RPDTree:BiTreeBase<Box>
    {
        protected int _maxCount;
        protected int _maxDepth;
        protected Mesh _mesh;

        private bool _preAllocateSFace = true;
        private bool _triByExterior = false;

        public RPDTree(Mesh mesh)
        {
            _name = "RPD";

            _maxCount = 100;
            _maxDepth = 20;
            _mesh = mesh;

            root = new BiNode<Box>(new Box(_mesh), 0);
            Create(root);
        }

        private void Create(BiNode<Box> parent)
        {
            if (parent == null)
                return;

            if (parent.value.faces.Count > _maxCount && parent.depth < _maxDepth)
            {  
                if (_triByExterior)
                {
                    var boundary = MeshUtility.FindExteriorEdge(parent.value.faces);
                    if (_preAllocateSFace)
                        parent.value.sFaces = MeshUtility.TriangulateGreedy(boundary, _mesh);//pre allocate
                    if (boundary.Count > parent.value.faces.Count)
                        return;
                }
                else
                {
                    var boundary = MeshUtility.FindHole(parent.value.faces);
                    if (_preAllocateSFace)
                        parent.value.sFaces = MeshUtility.TriangulateGreedy(boundary, _mesh);//pre allocate
                    if (boundary.Count > parent.value.faces.Count)
                        return;
                }

                double[] diag = MeshUtility.To3(parent.value.Diag);
                double max = double.MinValue;
                int segId = -1;
                for (int i = 0; i < 3; i++)
                {
                    if (diag[i] > max)
                    {
                        max = diag[i];
                        segId = i;
                    }
                }
                double[] center = MeshUtility.To3(parent.value.Center);

                List<IndexTriangle> lFaces = new List<IndexTriangle>();
                List<IndexTriangle> rFaces = new List<IndexTriangle>();
                foreach (var item in parent.value.faces)
                {
                    var tc = MeshUtility.To3(MeshUtility.TriangleCenter(item, _mesh));

                    if (tc[segId] < center[segId])
                    {
                        lFaces.Add(item);
                    }
                    else
                    {
                        rFaces.Add(item);
                    }
                }

                if (lFaces.Count > 0)
                {
                    parent.left = new BiNode<Box>(new Box(lFaces, _mesh), parent.depth + 1);
                    Create(parent.left);
                }
                if (rFaces.Count > 0)
                {
                    parent.right = new BiNode<Box>(new Box(rFaces, _mesh), parent.depth + 1);
                    Create(parent.right);
                }
            }
        }

        public override void Add()
        {
            throw new NotImplementedException();
        }

    }



    class WindingNumberRPD : IRelativePositionDetectionBase
    {
        public Mesh _mesh;
        protected double _threshold;

        public double Threshold { get { return _threshold; } set { _threshold = value; } }

        public virtual  void Init(Mesh mesh)
        {
            _mesh = mesh;
            _threshold = 0.5;
        }

        public  virtual bool IsInside(Point3D p)
        {
            return DetectProb(p) < _threshold;
        }

        public virtual double DetectProb(Point3D p)
        {
            var mesh = _mesh;
            const int dim = 3;

            double solidAngleSum = 0;
            foreach (var t in mesh.Triangles)
            {
                int[] vs = new int[dim] { t.V1, t.V2, t.V3 };
                Point3D[] ps = new Point3D[dim];
                for (int i = 0; i < dim; i++)
                    ps[i] = mesh.Vertices[vs[i]];
                var sa = MeshUtility.SolidAngle(ps, p);
                solidAngleSum += sa;
            }

            return solidAngleSum / (4 * Math.PI);
        }

        public static double DetectProb(IList<Triangle> faces,Point3D p)
        {      
            double solidAngleSum = 0;

            if (faces == null)
                return solidAngleSum;

            foreach (var t in faces)
            {
                var sa = MeshUtility.SolidAngle(t.Vertices, p);
                solidAngleSum += sa;
            }

            return solidAngleSum / (4 * Math.PI);
        }

        public static double DetectProb(IList<IndexTriangle> faces, Mesh mesh, Point3D p, ViewportLayout eyeshot = null)
        {
            double solidAngleSum = 0;

            if (faces == null)
                return solidAngleSum;

            foreach (var f in faces)
            {
                Triangle t = new Triangle(mesh.Vertices[f.V1], mesh.Vertices[f.V2], mesh.Vertices[f.V3]);
                var sa = MeshUtility.SolidAngle(t.Vertices, p);
                solidAngleSum += sa;
            }

            return solidAngleSum / (4 * Math.PI);
        }

        public static double NaiveWN(IList<Triangle> faces, Point3D p, ViewportLayout eyeshot = null)
        {
            return DetectProb(faces, p);
        }

        public static double NaiveWN(IList<IndexTriangle> faces, Mesh mesh, Point3D p,bool useSFaces, ViewportLayout eyeshot=null)
        {
            if(useSFaces)
            {
                var boundary = MeshUtility.FindHole(faces);
                var tris = MeshUtility.TriangulateGreedy(boundary, mesh);
                return DetectProb(tris, p);
            }
            else
            {
                return DetectProb(faces, mesh, p);
            }
        }
    }


    class WindingNumberRPD_Tree: WindingNumberRPD
    {
        public RPDTree _tree;
        private Triangle[] _sFaces;

        public override void Init(Mesh mesh)
        {
            base.Init(mesh);
            _tree = new RPDTree(mesh);
            InitSupplementTris(mesh);
        }

        public override double DetectProb(Point3D p)
        {
            return HierarchyWN(p, _tree.root);// + DetectSWN(p);
        }

        private void InitSupplementTris(Mesh m)
        {
            var hole = MeshUtility.FindHole(m.Triangles);
            var tris = MeshUtility.TriangulateGreedy(hole, m);
            _sFaces = tris.ToArray();
        }

        private double DetectSWN(Point3D p)
        {
            return DetectProb(_sFaces, p);
        }

        private double HierarchyWN(Point3D p,BiNode<Box> parent)
        {
            double w = 0;
            if (parent != null)
            {
                if (parent.IsLeaf)
                {
                    w = NaiveWN(parent.value.faces, _mesh, p,false);
                }
                else if (parent.value.IsInside(p))
                {
                    var wL = HierarchyWN(p, parent.left);
                    var wR = HierarchyWN(p, parent.right);
                    w = wL + wR;
                }
                else
                {
                    w = parent.value.sFaces.Count > 0 ? -1.0 * NaiveWN(parent.value.sFaces, p) : -1.0 * NaiveWN(parent.value.faces, _mesh, p, true);
                }
            }
            return w;
        }
    }

    class RayShootingRPD : IRelativePositionDetectionBase
    {
        public  void Init(Mesh mesh)
        {
            throw new NotImplementedException();
        }

        public  bool IsInside(Point3D p)
        {
            throw new NotImplementedException();
        }
    }

    class ClosetFaceNormalRPD : IRelativePositionDetectionBase
    {
        public  void Init(Mesh mesh)
        {
            throw new NotImplementedException();
        }

        public  bool IsInside(Point3D p)
        {
            throw new NotImplementedException();
        }
    }

    class RPDFactory
    {
        public enum RPDType
        {
            WindingNumber,
            RayShooting,
            ClosetFaceNormal
        }
        public static IRelativePositionDetectionBase Create(RPDType type)
        {
            switch (type)
            {
                case RPDType.WindingNumber:
                    return new WindingNumberRPD();
                case RPDType.RayShooting:
                    return new RayShootingRPD();
                case RPDType.ClosetFaceNormal:
                    return new ClosetFaceNormalRPD();
                default:
                    return null;
            }
        }
    }

    class DS_Algo<T> where T : IComparable
    {
        public static List<T> st_1(List<T> data)
        {
            List<T> res = new List<T>();
            res.Add(data[0]);

            while (res.Count != data.Count)
            {
                bool flag = false;
                T curr = data[res.Count];
                for (int i = res.Count - 1; i >= 0; --i)
                {
                    if (curr.CompareTo(res[i]) >= 0)
                    {
                        res.Insert(i + 1, curr);
                        flag = true;
                        break;
                    }
                }
                if (!flag) res.Insert(0, curr);
            }

            return res;
        }

        public static List<T> st_2(List<T> data)
        {
            List<T> res = new List<T>();
            res.AddRange(data);
            st_2_base(res, 0, res.Count - 1);
            return res;
        }

        public static List<T> st_3(List<T> data)
        {
            List<T> res = new List<T>();
            res.AddRange(data);
            st_3_base(res, 0, res.Count - 1);
            return res;
        }

        public static List<T> st_4(List<T> data)
        {
            MaxHeap<T> mh = new MaxHeap<T>(data);

            for (int i = 1; i < mh.Data.Count; i++)
            {
                Swap(mh.Data, 0, mh.Data.Count - i);
                mh.Heapify(0, mh.Data.Count - i);
            }

            return mh.Data.ToList();
        }

        public static List<int> st_5(List<int> data)
        {
            List<int> res = new List<int>();
            res.AddRange(data);

            var max = Max(data);
            var min = Min(data);

            List<KeyValuePair<int, int>> counter = new List<KeyValuePair<int, int>> ();

            for (int i = min; i <= max; i++)
                counter.Add(new KeyValuePair<int, int>(i, 0));

            for (int i = 0; i < data.Count; i++)
                counter[data[i] - min] = new KeyValuePair<int, int>(data[i], counter[data[i] - min].Value + 1);

            for (int i = 1; i < counter.Count; i++)
                counter[i] = new KeyValuePair<int, int>(counter[i].Key, counter[i].Value + counter[i - 1].Value);

            for (int i = 0; i < counter.Count; i++)
                counter[i] = new KeyValuePair<int, int>(counter[i].Key, counter[i].Value -1);

            for (int i = data.Count - 1; i >=0 ; i--)
            {
                var val = data[i];
                var id = val - min;

                res[counter[id].Value] = counter[id].Key;
                counter[id] = new KeyValuePair<int, int>(counter[id].Key, counter[id].Value - 1);

            }

            return res;
        }

        public static ScalarType  Max<ScalarType>(List<ScalarType> data) where ScalarType : IComparable
        {
            var max = data[0];

            for (int i = 1; i < data.Count; i++)
            {
                if(max.CompareTo(data[i])<0)
                {
                    max = data[i];
                }
            }

            return max;
        }

        public static ScalarType Min<ScalarType>(List<ScalarType> data) where ScalarType : IComparable
        {
            var min = data[0];

            for (int i = 1; i < data.Count; i++)
            {
                if (min.CompareTo(data[i]) > 0)
                {
                    min = data[i];
                }
            }

            return min;
        }

        public static void Swap(ref T a,ref T b)
        {
            T tmp = a;
            a = b;
            b = tmp;
        }

        private static void Swap(List<T> data, int i, int j)
        {
            T tmp = data[i];
            data[i] = data[j];
            data[j] = tmp;
        }

        private static void st_2_base(List<T> data,int beg,int end)
        {
            if (beg == end)
                return;

            if(beg+1==end)
            {
                if(data[beg].CompareTo(data[end])>0)
                {
                    Swap(data, beg, end);
                }
                return;
            }

            var mid = (beg + end) >> 1;

            st_2_base(data, beg, mid);
            st_2_base(data, mid+1, end);

            List<T> tmp = new List<T>();
            int il = beg, ir = mid+1;
            while (il<=mid&&ir<=end)
            {
                if(data[il].CompareTo(data[ir])<=0)
                {
                    tmp.Add(data[il++]);
                }
                else
                {
                    tmp.Add(data[ir++]);
                }
            }
            for (int i = il; i <= mid; i++)
            {
                tmp.Add(data[i]);
            }
            for (int i = ir; i <= end; i++)
            {
                tmp.Add(data[i]);
            }

            for (int i = beg; i <=end; i++)
            {
                data[i] = tmp[i - beg];
            }

        }

        private static void st_3_base(List<T> data, int beg, int end)
        {
            if (beg >= end)
                return;

            if (beg + 1 == end)
            {
                if (data[beg].CompareTo(data[end]) > 0)
                {
                    Swap(data, beg, end);
                }
                return;
            }

            var pivot = data[end]; 

            var tail = beg;
            for (int i = beg; i <end; i++)
            {
                if(data[i].CompareTo(pivot)<=0)
                {
                    Swap(data, i, tail++);
                }
            }
            Swap(data, end, tail);

            st_3_base(data, beg, tail-1);
            st_3_base(data, tail+1, end);
        }
    }
    

}



