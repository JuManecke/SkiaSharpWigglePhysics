using SkiaSharp;                               // is the 2D renderer framework I use
using SkiaSharp.Views.Desktop;                 // enables the draw-function with Skia
using System;                                  
using System.Collections.Generic;              // to use dynamic containers (List, Dictionary, etc.)
using System.Linq;                             // needed for the bottom-row pinning
using System.Windows.Forms;                    // WinForms UI to use the windows-window 
using Wiggle.Core;
using Wiggle.Core.Presets;

// General idea:      A softbody mesh made of triangles, in the shape of a goose, that jiggles when the window moves or when poked/dragged with the mouse.
// General behaviour: The mesh jiggles when the window moves, the triangles can be clicked to poke them or can be dragged around softly.
//                    The bottom row of the mesh is pinned to keep the body upright and the triangles should not flip or collapse.
// General structure: The mesh is made of vertices, edges, and triangles. Every vertex is treated as a particle and uses verlet integration.
//                    The verlet integration is done in the SoftBody class, which also handles the physics, movement and drawing of the mesh.
//                    The mesh keeps its general shape with distance constraints on the edges, and a few long-range edges to prevent collapse or overstretching.
//                    A no-flip constraint keeps triangles from inverting or collapsing too thin and a tether pulls the mesh back to its default position when idle.
// How to use it?:    Move the window to make the goose jiggle, click triangles to poke them, and drag triangles to move them around.
//                    Have fun! Honk! Honk! Honk!

// Basis:             Vector2 class with + - * / length normal dot 90°-perp is needed for the physics
//                    Triangle class to hold the body with its color
//                    Vertex class to hold the points of the mesh
//                    Edge class to hold the edges of the mesh
//                    TriangleIndex class to hold the triangle indices and orientation references
//                    SoftBody class to hold the mesh, run the physics, and draw the body
//                    GooseForm class to hold the window, canvas, and input handling

// At start:          The window is created with a Skia canvas inside it.
//                    The goose shape is defined as a list of triangles with their colors.

// The mesh:          The SoftBody is created from the triangles, building the vertices and edges, and pinning the bottom row.
//                    Vertices of the triangles that are identical in position are shared to avoid duplicates.
//                    Triangles are stored with their vertex indices and orientation references for no-flip constraints.
//                    Edges are created for each triangle edge as distance constraints.
//                    Long-range edges are added to keep the overall shape and reduce large-scale sagging.

// At runtime:        The window movement gives acceleration into the sim to create a jiggle.
//                    The canvas redraws on a timer, running the physics step and drawing the mesh.
//                    The physics uses verlet integration, frame collisions and simple self-collision.
//                    Distance constraints keep edge lengths, no-flip keeps orientation/thickness, tether returns to rest when idle.
//                    Mouse input lets me click to poke and drag triangles softly.

// THE OLD CODE:
/*
namespace GooseWiggle
{
    public static class Program
    {
        [STAThread]                             // WinForms requires STA to run, else it wont start
        public static void Main() 
        {
            ApplicationConfiguration.Initialize(); // WinForms UI initialization to draw everything on it
            Application.Run(new GooseForm());      // Start message loop with our main window — blocks until window closes
        }
    }

    // own vector 2D class for the physics calculations
    public struct Vec2
    {
        public float X, Y;               

        public Vec2(float x, float y) { X = x; Y = y; } // constructor 

        public static Vec2 operator + (Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
        public static Vec2 operator - (Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
        public static Vec2 operator * (Vec2 a, float s) => new(a.X * s, a.Y * s); 
        public static Vec2 operator * (float s, Vec2 a) => new(a.X * s, a.Y * s); 
        public static Vec2 operator / (Vec2 a, float s) => new(a.X / s, a.Y / s);

        public float Length => MathF.Sqrt(X * X + Y * Y);
        public float LengthSq => X * X + Y * Y;           // Squared length is cheaper when comparing distances

        public Vec2 Normalized()
        {
            float length = Length;
            if (length > .000000000001f)
                return new Vec2(X / length, Y / length);
            else
                return new Vec2(0, 0);
        }

        public static float Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y; // dot product — projection along a direction
        public static Vec2 PerpCCW(Vec2 v) => new Vec2(-v.Y, v.X); // 90° counter-clockwise perpendicular vector
    }
    
    public class Triangle
    {
        public SKPoint A, B, C;                 
        public SKColor FillColor;               

        public Triangle(SKPoint a, SKPoint b, SKPoint c, SKColor color) // constructor
        {
            A = a; 
            B = b; 
            C = c; 
            FillColor = color;
        }
    }
    
    public class Vertex
    {
        public Vec2 CurrentPosition;                   
        public Vec2 PreviousPosition;           
        public Vec2 DefaultPosition;            // pulls back into this position when idle
        public float Mass = 1f;                 
        public bool IsPinned = false;           // bottom row is pinned as an anchor
        public float Radius = 5f;               // radius for vertex on vertex self-collision
    }
    
    public class Edge
    {
        public int I, J;                        // vertex indices of the edge (endpoints A and B).
        public float DefaultLength;             // length of  the DefaultPosition
        public float Stiffness;                 // of the edges
    }
    
    public class TriangleIndex
    {
        public int A, B, C;                     
        public SKColor Fill;                    
        
        public float DefaultHeight_C_AB;        // Signed distances of first point to line in the Default position
        public float DefaultHeight_A_BC;        
        public float DefaultHeight_B_CA;        

        public float MinHeightFraction = 0.35f; // Minimum height to prevent collapse or flip of my triangles
    }
    
    // The softbody builds the mesh, runs physics, and draws the goose body
    public class SoftBody
    {
        public List<Vertex> Vertices = new();     // list of all points in the mesh
        public List<Edge> Edges = new();          // list of edges to keep the shape
        public List<TriangleIndex> Triangles = new();

        // Variables for adaptability
        public float VelocityDamping    = 0.990f;           // slows velocity to prevents endless jiggling - lower = less wobble, higher = bouncier
        public float EdgeStiffness      = 0.85f;            // Stiffness for the edge distances of all triangles - higher = firmer shape, lower = softer
        public int UpdateAtWhichFPS     = 25;               // Iterations per frame
        public float WallBounce         = 0.8f;             
        public float DefaultPositionPullStrength = 0.035f; 
        public float WindowShakeStrength = 1.6f;            // how much does the window movement shake the mesh

        // Long-range additions to keep overall shape
        public float LongRangeStiffness = 0.18f;            // needed to prevent triangles from collapsing into each other
        public int LongRangeMaxNeighbors = 2;               // Max neighbors per edge
        public float LongRangeMaxDistance = 220f;           // to avoid overlapping triangles

        // click jiggle + soft triangle drag
        public float ClickImpulseStrength = 24f;           
        public bool IsDragging = false;                    
        public int DragTriangleIndex = -1;                 // Which triangle is being dragged
        public float DragWeightU, DragWeightV, DragWeightW; // mouse-down grip point  to keep it stable during dragging
        public Vec2 DragTarget;                            // Mouse position in the canvas space
        public float DragStiffness = 0.6f;                 // 0-1 scale fraction - higher = more drag.

        public SKRect Bounds;                              // Simulation rectangle inside the frame — used for wall collisions

        /// <summary>
        /// build the softbody from triangles: share vertices, store triangle indices + rest heights,
        /// add distance edges + a few long-range edges, pin the bottom row.
        /// Source: https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/14-cloth.html via https://matthias-research.github.io/pages/tenMinutePhysics/
        /// </summary>
        public static SoftBody FromTriangles(List<Triangle> input, SKRect bounds)
        {
            var body = new SoftBody { Bounds = bounds };          // Creating a instance to reference

            var indexOf = new Dictionary<(int X, int Y), int>();  // canvas-coordinates (x,y) - vertex index

            int GetOrCreateVertex(SKPoint vertexPoint)            // returns index of the shared vertex or else creates a new one
            {
                var key = (X: (int)MathF.Round(vertexPoint.X),              
                           Y: (int)MathF.Round(vertexPoint.Y));             

                if (!indexOf.TryGetValue(key, out int idx))       // if not seen before, create a new vertex
                {
                    var v = new Vertex
                    {
                        CurrentPosition     = new Vec2(key.X, key.Y),        
                        PreviousPosition    = new Vec2(key.X, key.Y),        
                        DefaultPosition     = new Vec2(key.X, key.Y),        
                        Radius   = 5f                             // self-collision radius
                    };
                    idx = body.Vertices.Count;                    // New index is current count
                    body.Vertices.Add(v);                         // Storing the vertex
                    indexOf[key] = idx;                           // save it for next time this coordinate appears
                }
                return idx;                                       // Return index whether found or created
            }

            foreach (var t in input)                       // foreach triangle in the list
            {
                int a = GetOrCreateVertex(t.A);                   // Get the points of the triangle
                int b = GetOrCreateVertex(t.B);                    
                int c = GetOrCreateVertex(t.C);                    

                var A0 = body.Vertices[a].DefaultPosition;   // set the default positions of the points                 
                var B0 = body.Vertices[b].DefaultPosition;                    
                var C0 = body.Vertices[c].DefaultPosition;                    

                float hC = GetHeight(C0, A0, B0);               // Height of the points
                float hA = GetHeight(A0, B0, C0);           
                float hB = GetHeight(B0, C0, A0);           

                body.Triangles.Add(new TriangleIndex             // Add triangle to the list
                {
                    A = a, B = b, C = c,                           // Indixes of the triangle points
                    Fill = t.FillColor,                            // Color for drawing
                    DefaultHeight_C_AB = hC,                       // Orientation refs
                    DefaultHeight_A_BC = hA,
                    DefaultHeight_B_CA = hB,
                    MinHeightFraction = 0.35f                      // Minimum height fraction to prevent collapse or flip
                });
            }

            var edgeSet = new HashSet<(int, int)>();               
            void AddEdgeIfNew(int i, int j, float stiffness)       // add constraint edge if not already present
            {
                if (i == j) return;                                //  Ignore self-pairs
                if (i > j) (i, j) = (j, i);                        //  Order pair (small,big)
                if (edgeSet.Add((i, j)))                           // If newly inserted, the position gets set
                {
                    float defaultLength = (body.Vertices[j].CurrentPosition - body.Vertices[i].CurrentPosition).Length;
                    body.Edges.Add(new Edge { I = i, J = j, DefaultLength = defaultLength, Stiffness = stiffness }); 
                }
            }
            foreach (var tri in body.Triangles)         // For all triangle edges
            {
                AddEdgeIfNew(tri.A, tri.B, body.EdgeStiffness); 
                AddEdgeIfNew(tri.B, tri.C, body.EdgeStiffness); 
                AddEdgeIfNew(tri.C, tri.A, body.EdgeStiffness); 
            }

            // adding a few long-range edges to keep the overall form of the body
            int n = body.Vertices.Count;
            for (int i = 0; i < n; i++)
            {
                var pi = body.Vertices[i].CurrentPosition;
                var candidates = new List<(float d, int j)>(n - 1);

                for (int j = 0; j < n; j++)
                {
                    if (i == j) continue;
                    float d = (body.Vertices[j].CurrentPosition - pi).Length;
                    if (d <= body.LongRangeMaxDistance) candidates.Add((d, j));
                }

                candidates.Sort((a, b) => a.d.CompareTo(b.d));
                int added = 0;

                foreach (var (d, j) in candidates)
                {
                    if (added >= body.LongRangeMaxNeighbors) break;
                    int a = i, b = j; if (a > b) (a, b) = (b, a);
                    if (edgeSet.Contains((a, b))) continue;

                    edgeSet.Add((a, b));
                    body.Edges.Add(new Edge
                    {
                        I = i, J = j, DefaultLength = d, Stiffness = body.LongRangeStiffness
                    });
                    added++;
                }
            }

            // pin the bottom row as the anchor
            float maxY = body.Vertices.Max(v => v.CurrentPosition.Y);
            foreach (var v in body.Vertices)
                if (MathF.Abs(v.CurrentPosition.Y - maxY) < 0.5f)
                    v.IsPinned = true;

            return body;
        }

        /// <summary>height of a point of an edge - needed for the not-flipping.</summary>
        private static float GetHeight(in Vec2 P, in Vec2 A, in Vec2 B)
        {
            Vec2 ab = B - A;
            Vec2 n = Vec2.PerpCCW(ab).Normalized();
            return Vec2.Dot(P - A, n);
        }

        /// <summary>searching for the triangle at the current position</summary>
        public int FindTriangleAtPoint(Vec2 p)
        {
            for (int i = 0; i < Triangles.Count; i++)
            {
                var tr = Triangles[i];
                var a = Vertices[tr.A].CurrentPosition;
                var b = Vertices[tr.B].CurrentPosition;
                var c = Vertices[tr.C].CurrentPosition;
                if (PointInTriangle(p, a, b, c)) return i;
            }
            return -1;
        }

        /// <summary>inside test using barycentric coordinates.</summary>
        public static bool PointInTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c)
        {
            Vec2 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = Vec2.Dot(v0, v0);
            float d01 = Vec2.Dot(v0, v1);
            float d11 = Vec2.Dot(v1, v1);
            float d20 = Vec2.Dot(v2, v0);
            float d21 = Vec2.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (MathF.Abs(denom) < 1e-6f) return false; // == ca. 0.00000000001f
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1f - v - w;
            return (u >= 0f && v >= 0f && w >= 0f);
        }

        /// <summary>
        /// barycentric weights of a point in a triangle
        /// three numbers summing to 1 to locate a point in the triangle and make dragging clean and stable
        ///
        /// used Source: https://math.stackexchange.com/questions/1887215/fast-way-of-computing-barycentric-coordinates-explained?
        /// </summary>
        private static (float u, float v, float w) Barycentric(Vec2 p, Vec2 a, Vec2 b, Vec2 c)
        {
            Vec2 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = Vec2.Dot(v0, v0);
            float d01 = Vec2.Dot(v0, v1);
            float d11 = Vec2.Dot(v1, v1);
            float d20 = Vec2.Dot(v2, v0);
            float d21 = Vec2.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (MathF.Abs(denom) < 1e-6f) return (1, 0, 0);
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1f - v - w;
            return (u, v, w);
        }

        /// <summary> click impulse on the points of that triangle.</summary>
        public void ApplyClickImpulseToTriangle(int triIndex, Vec2 clickPos, float strength)
        {
            if (triIndex < 0 || triIndex >= Triangles.Count) return;
            var tr = Triangles[triIndex];

            void Move(int idx)
            {
                var v = Vertices[idx];
                if (v.IsPinned) return;
                Vec2 dir = (v.CurrentPosition - clickPos);
                if (dir.LengthSq < 1e-6f) dir = new Vec2(0.001f, 0);
                dir = dir.Normalized();
                v.PreviousPosition -= dir * strength;
                Vertices[idx] = v;
            }

            Move(tr.A); Move(tr.B); Move(tr.C);
        }

        /// <summary>when beginning to drag a triangle: store index and barycentric weights</summary>
        public void StartDragTriangle(int triangleIndex, Vec2 mouse)
        {
            if (triangleIndex < 0 || triangleIndex >= Triangles.Count) return;
            IsDragging = true;
            DragTriangleIndex = triangleIndex;

            var tr = Triangles[triangleIndex];
            var a = Vertices[tr.A].CurrentPosition;
            var b = Vertices[tr.B].CurrentPosition;
            var c = Vertices[tr.C].CurrentPosition;

            (DragWeightU, DragWeightV, DragWeightW) = Barycentric(mouse, a, b, c);
            DragTarget = mouse;
        }
        
        public void UpdateDrag(Vec2 target)
        {
            if (!IsDragging) return;
            DragTarget = target;
        }
        
        public void EndDrag()
        {
            IsDragging = false;
            DragTriangleIndex = -1;
        }
        
        /// <summary>
        /// Source: https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/08-interaction.html via https://matthias-research.github.io/pages/tenMinutePhysics/
        /// </summary>
        private void ApplySoftDrag()
        {
            if (!IsDragging || DragTriangleIndex < 0 || DragTriangleIndex >= Triangles.Count) return;

            var triangle = Triangles[DragTriangleIndex];
            var pointA = Vertices[triangle.A].CurrentPosition;
            var pointB = Vertices[triangle.B].CurrentPosition;
            var pointC = Vertices[triangle.C].CurrentPosition;

            Vec2 handle = DragWeightU * pointA + DragWeightV * pointB + DragWeightW * pointC;
            Vec2 error = DragTarget - handle;
            Vec2 moveA = error * (DragStiffness * DragWeightU);
            Vec2 moveB = error * (DragStiffness * DragWeightV);
            Vec2 moveC = error * (DragStiffness * DragWeightW);

            if (!Vertices[triangle.A].IsPinned) { Vertices[triangle.A].CurrentPosition += moveA; Vertices[triangle.A].PreviousPosition += moveA; }
            if (!Vertices[triangle.B].IsPinned) { Vertices[triangle.B].CurrentPosition += moveB; Vertices[triangle.B].PreviousPosition += moveB; }
            if (!Vertices[triangle.C].IsPinned) { Vertices[triangle.C].CurrentPosition += moveC; Vertices[triangle.C].PreviousPosition += moveC; }
        }

        /// <summary>
        /// moves vertices → applys drag → keeps it in the frame → enforces edge length → prevents triangle flipping.
        /// Source: https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/10-softBodies.html via https://matthias-research.github.io/pages/tenMinutePhysics/
        /// </summary> 
        public void SimulationSteps(float dt, Vec2 windowAcceleration)
        {
            if (dt <= 0f || dt > 0.1f) dt = 1f / 60f;
            float dt2 = dt * dt;

            // verlet integration + window wiggle
            for (int i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                if (v.IsPinned) continue;

                Vec2 velocity = (v.CurrentPosition - v.PreviousPosition) * VelocityDamping;
                Vec2 accel = new Vec2(-windowAcceleration.X, -windowAcceleration.Y) * WindowShakeStrength;
                Vec2 next = v.CurrentPosition + velocity + accel * dt2;

                v.PreviousPosition = v.CurrentPosition;
                v.CurrentPosition  = next;

                Vec2 toRest = v.DefaultPosition - v.CurrentPosition;
                v.CurrentPosition += toRest * DefaultPositionPullStrength;

                Vertices[i] = v;
            }
            
            ApplySoftDrag();

            // frame collision handling
            for (int i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                if (v.IsPinned) continue;

                Vec2 vel = v.CurrentPosition - v.PreviousPosition;

                if (v.CurrentPosition.X < Bounds.Left)
                {
                    v.CurrentPosition.X = Bounds.Left;
                    v.PreviousPosition.X = v.CurrentPosition.X + (-vel.X) * WallBounce;
                }
                if (v.CurrentPosition.X > Bounds.Right)
                {
                    v.CurrentPosition.X = Bounds.Right;
                    v.PreviousPosition.X = v.CurrentPosition.X + (-vel.X) * WallBounce;
                }
                if (v.CurrentPosition.Y < Bounds.Top)
                {
                    v.CurrentPosition.Y = Bounds.Top;
                    v.PreviousPosition.Y = v.CurrentPosition.Y + (-vel.Y) * WallBounce;
                }
                if (v.CurrentPosition.Y > Bounds.Bottom)
                {
                    v.CurrentPosition.Y = Bounds.Bottom;
                    v.PreviousPosition.Y = v.CurrentPosition.Y + (-vel.Y) * WallBounce;
                }

                Vertices[i] = v;
            }

            // self-collision
            for (int i = 0; i < Vertices.Count; i++)
            {
                for (int j = i + 1; j < Vertices.Count; j++)
                {
                    var a = Vertices[i]; var b = Vertices[j];
                    float minDist = a.Radius + b.Radius;
                    Vec2 delta = b.CurrentPosition - a.CurrentPosition;
                    float d2 = delta.LengthSq;
                    if (d2 < minDist * minDist && d2 > 1e-6f)
                    {
                        float d = MathF.Sqrt(d2);
                        Vec2 n = delta / d;
                        float overlap = (minDist - d);

                        float ma = a.IsPinned ? 0f : a.Mass;
                        float mb = b.IsPinned ? 0f : b.Mass;
                        float sum = ma + mb; if (sum <= 0) continue;

                        float moveA = (mb / sum) * overlap;
                        float moveB = (ma / sum) * overlap;

                        if (!a.IsPinned) a.CurrentPosition -= n * moveA;
                        if (!b.IsPinned) b.CurrentPosition += n * moveB;

                        Vertices[i] = a; Vertices[j] = b;
                    }
                }
            }

            // keep edge distance - position-based dynamic (PBD)
            for (int it = 0; it < UpdateAtWhichFPS; it++)
            {
                foreach (var e in Edges)
                {
                    var a = Vertices[e.I]; var b = Vertices[e.J];
                    Vec2 delta = b.CurrentPosition - a.CurrentPosition;
                    float dist = delta.Length; if (dist < 1e-6f) continue;

                    float diff = (dist - e.DefaultLength) / dist;
                    Vec2 correction = delta * (0.5f * e.Stiffness * diff);

                    if (!a.IsPinned) a.CurrentPosition += correction;
                    if (!b.IsPinned) b.CurrentPosition -= correction;

                    Vertices[e.I] = a; Vertices[e.J] = b;
                }
                ApplySoftDrag();
            }

            // keep orientation and minimum thickness to prevetn flipping
            foreach (var tr in Triangles)
            {
                EnforceSignedHeight(tr.A, tr.B, tr.C, tr.DefaultHeight_C_AB, tr.MinHeightFraction);
                EnforceSignedHeight(tr.B, tr.C, tr.A, tr.DefaultHeight_A_BC, tr.MinHeightFraction);
                EnforceSignedHeight(tr.C, tr.A, tr.B, tr.DefaultHeight_B_CA, tr.MinHeightFraction);
            }
        }

        /// <summary>prevent inwards rotation of the triangle</summary>
        private void EnforceSignedHeight(int i, int j, int k, float restHeight, float minFrac)
        {
            var A = Vertices[i]; var B = Vertices[j]; var C = Vertices[k];

            Vec2 ab = B.CurrentPosition - A.CurrentPosition;
            float abLen = ab.Length; if (abLen < 1e-6f) return;
            Vec2 n = Vec2.PerpCCW(ab) / abLen;
            float d = Vec2.Dot(C.CurrentPosition - A.CurrentPosition, n);

            float targetSign = MathF.Sign(restHeight);
            float minMag = MathF.Abs(restHeight) * minFrac;

            bool wrongSide = MathF.Sign(d) != targetSign;
            bool tooThin   = MathF.Abs(d) < minMag;

            if (wrongSide || tooThin)
            {
                float desired = targetSign * MathF.Max(minMag, MathF.Abs(d));
                float corr = desired - d;

                if (!C.IsPinned)
                {
                    C.CurrentPosition += n * corr;
                }
                else
                {
                    if (!A.IsPinned) A.CurrentPosition -= n * (corr * 0.5f);
                    if (!B.IsPinned) B.CurrentPosition -= n * (corr * 0.5f);
                }

                Vertices[i] = A; Vertices[j] = B; Vertices[k] = C;
            }
        }

        /// <summary>draw the mesh, color the triangle, put outlines on it</summary>
        public void DrawMesh(SKCanvas canvas)
        {
            using var stroke = new SKPaint { Color = SKColors.Black, IsStroke = true, StrokeWidth = 2, IsAntialias = true };

            foreach (var tri in Triangles)
            {
                var a = Vertices[tri.A].CurrentPosition;
                var b = Vertices[tri.B].CurrentPosition;
                var c = Vertices[tri.C].CurrentPosition;

                using var fill = new SKPaint { Color = tri.Fill, IsAntialias = true };
                using var path = new SKPath();
                path.MoveTo(a.X, a.Y);
                path.LineTo(b.X, b.Y);
                path.LineTo(c.X, c.Y);
                path.Close();

                canvas.DrawPath(path, fill);
                canvas.DrawPath(path, stroke);
            }
        }
    }
    
    public class GooseForm : Form
    {
        private readonly SKControl canvasControl;                   // SkiaSharp stuff
        private readonly List<Triangle> sourceTriangles = new();    // list of the triangles i used for the body
        private SoftBody body;                                      // The simulated mesh softbody

        private readonly Timer frameTimer;                          // FPS timer
        private DateTime lastFrameTime;                             

        private Vec2 previousWindowPosition;                        
        private Vec2 previousWindowVelocity;                     

        private const int FrameStrokeWidth = 20;                    //  Red border thickness
        private const int InnerPadding = 0;                         //  Inner padding inside red border

        /// <summary>setup: create canvas, load triangles, build softbody.</summary>
        public GooseForm()
        {
            Text = "Goose Triangle-Softbody 2D – Wiggle Physics";
            ClientSize = new System.Drawing.Size(820, 820);

            canvasControl = new SKControl { Dock = DockStyle.Fill };
            canvasControl.PaintSurface += OnPaintSurface;           // getting the events in
            canvasControl.MouseDown += OnMouseDown;
            canvasControl.MouseMove += OnMouseMove;
            canvasControl.MouseUp   += OnMouseUp;
            canvasControl.MouseLeave += (_, __) => body?.EndDrag(); // using _ and __ to ignore unused parameters

            Controls.Add(canvasControl);

            InitializeTriangles();

            body = SoftBody.FromTriangles(sourceTriangles, InnerBoundsRectangle());

            lastFrameTime = DateTime.Now;
            previousWindowPosition = WindowScreenPosition();

            frameTimer = new Timer { Interval = 16 };
            frameTimer.Tick += (_, __) => TickFrame();
            frameTimer.Start();
        }

        /// <summary> goose-body triangles, positions and colors</summary>
        private void InitializeTriangles()
        {
            sourceTriangles.Clear();

            var orange = SKColors.Orange;
            var white  = SKColors.White;
            var black  = SKColors.Black;

            // BEAK (orange) 
            sourceTriangles.Add(new Triangle(new SKPoint(140, 450), new SKPoint(220, 460), new SKPoint(240, 380), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(220, 460), new SKPoint(320, 400), new SKPoint(240, 380), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(240, 380), new SKPoint(320, 400), new SKPoint(310, 320), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(220, 460), new SKPoint(330, 450), new SKPoint(320, 400), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(330, 450), new SKPoint(400, 430), new SKPoint(320, 400), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(320, 400), new SKPoint(400, 430), new SKPoint(380, 340), orange));
            sourceTriangles.Add(new Triangle(new SKPoint(320, 400), new SKPoint(380, 340), new SKPoint(310, 320), orange));

            // HEAD (white)
            sourceTriangles.Add(new Triangle(new SKPoint(400, 430), new SKPoint(530, 430), new SKPoint(380, 340), white));
            sourceTriangles.Add(new Triangle(new SKPoint(380, 340), new SKPoint(495, 300), new SKPoint(380, 220), white));
            sourceTriangles.Add(new Triangle(new SKPoint(380, 340), new SKPoint(530, 430), new SKPoint(495, 300), white));
            sourceTriangles.Add(new Triangle(new SKPoint(310, 320), new SKPoint(380, 340), new SKPoint(380, 220), white));
            sourceTriangles.Add(new Triangle(new SKPoint(495, 300), new SKPoint(450, 268), new SKPoint(480, 160), white));
            sourceTriangles.Add(new Triangle(new SKPoint(380, 220), new SKPoint(450, 268), new SKPoint(480, 160), white));
            sourceTriangles.Add(new Triangle(new SKPoint(537, 260), new SKPoint(580, 220), new SKPoint(480, 160), white));
            sourceTriangles.Add(new Triangle(new SKPoint(487, 240), new SKPoint(537, 260), new SKPoint(480, 160), white));
            sourceTriangles.Add(new Triangle(new SKPoint(495, 300), new SKPoint(620, 300), new SKPoint(580, 220), white));
            sourceTriangles.Add(new Triangle(new SKPoint(495, 300), new SKPoint(530, 430), new SKPoint(620, 300), white));
            sourceTriangles.Add(new Triangle(new SKPoint(530, 430), new SKPoint(600, 460), new SKPoint(620, 300), white));
            sourceTriangles.Add(new Triangle(new SKPoint(530, 430), new SKPoint(560, 550), new SKPoint(600, 460), white));
            sourceTriangles.Add(new Triangle(new SKPoint(400, 430), new SKPoint(490, 510), new SKPoint(530, 430), white));
            sourceTriangles.Add(new Triangle(new SKPoint(490, 510), new SKPoint(560, 550), new SKPoint(530, 430), white));
            sourceTriangles.Add(new Triangle(new SKPoint(330, 450), new SKPoint(400, 480), new SKPoint(400, 430), white));
            sourceTriangles.Add(new Triangle(new SKPoint(400, 480), new SKPoint(490, 510), new SKPoint(400, 430), white));

            // EYE (black)
            sourceTriangles.Add(new Triangle(new SKPoint(450, 268), new SKPoint(495, 300), new SKPoint(487, 240), black));
            sourceTriangles.Add(new Triangle(new SKPoint(495, 300), new SKPoint(537, 260), new SKPoint(487, 240), black));

            // NECK (white)
            sourceTriangles.Add(new Triangle(new SKPoint(540, 625), new SKPoint(500, 625), new SKPoint(560, 550), white));
            sourceTriangles.Add(new Triangle(new SKPoint(500, 625), new SKPoint(560, 700), new SKPoint(540, 625), white));
            sourceTriangles.Add(new Triangle(new SKPoint(460, 700), new SKPoint(560, 700), new SKPoint(500, 625), white));
            sourceTriangles.Add(new Triangle(new SKPoint(500, 625), new SKPoint(560, 550), new SKPoint(490, 510), white));

            // BODY (white)
            sourceTriangles.Add(new Triangle(new SKPoint(290, 787), new SKPoint(350, 700), new SKPoint(320, 650), white));
            sourceTriangles.Add(new Triangle(new SKPoint(350, 700), new SKPoint(370, 590), new SKPoint(320, 650), white));
            sourceTriangles.Add(new Triangle(new SKPoint(370, 590), new SKPoint(490, 510), new SKPoint(400, 480), white));
            sourceTriangles.Add(new Triangle(new SKPoint(350, 700), new SKPoint(400, 630), new SKPoint(370, 590), white));
            sourceTriangles.Add(new Triangle(new SKPoint(350, 700), new SKPoint(460, 700), new SKPoint(400, 630), white));
            sourceTriangles.Add(new Triangle(new SKPoint(460, 700), new SKPoint(500, 625), new SKPoint(400, 630), white));
            sourceTriangles.Add(new Triangle(new SKPoint(400, 630), new SKPoint(500, 625), new SKPoint(490, 510), white));
            sourceTriangles.Add(new Triangle(new SKPoint(400, 630), new SKPoint(490, 510), new SKPoint(370, 590), white));

            sourceTriangles.Add(new Triangle(new SKPoint(350, 700), new SKPoint(400, 787), new SKPoint(460, 700), white));
            sourceTriangles.Add(new Triangle(new SKPoint(460, 700), new SKPoint(510, 787), new SKPoint(560, 700), white));

            // LOWER BODY (white, bottom static row)
            sourceTriangles.Add(new Triangle(new SKPoint(240, 787), new SKPoint(290, 787), new SKPoint(320, 650), white));
            sourceTriangles.Add(new Triangle(new SKPoint(290, 787), new SKPoint(400, 787), new SKPoint(350, 700), white));
            sourceTriangles.Add(new Triangle(new SKPoint(400, 787), new SKPoint(510, 787), new SKPoint(460, 700), white));
            sourceTriangles.Add(new Triangle(new SKPoint(510, 787), new SKPoint(620, 787), new SKPoint(560, 700), white));
        }

        /// <summary>FPS ticker</summary>
        private void TickFrame()
        {
            var now = DateTime.Now;
            float dt = (float)(now - lastFrameTime).TotalSeconds;
            lastFrameTime = now;

            Vec2 windowPos = WindowScreenPosition();
            Vec2 windowVel = (windowPos - previousWindowPosition) / Math.Max(dt, 1e-6f);
            Vec2 windowAcc = (windowVel - previousWindowVelocity) / Math.Max(dt, 1e-6f);
            previousWindowPosition = windowPos;
            previousWindowVelocity = windowVel;

            body.Bounds = InnerBoundsRectangle();

            body.SimulationSteps(dt, windowAcc);
            canvasControl.Invalidate();
        }

        /// <summary>painting the red border and the goose mesh</summary>
        private void OnPaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            var canvas = e.Surface.Canvas;
            canvas.Clear(new SKColor(45, 45, 45));

            using var borderPaint = new SKPaint
            {
                Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStrokeWidth, IsAntialias = true
            };
            canvas.DrawRect(0, 0, e.Info.Width, e.Info.Height, borderPaint);

            body.DrawMesh(canvas);
        }

        /// <summary>left click = poke + start drag, move = drag, release/leave = end drag</summary> 
        private void OnMouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || body == null) return;
            var p = new Vec2(e.X, e.Y);

            int tri = body.FindTriangleAtPoint(p);
            if (tri < 0) return;

            body.ApplyClickImpulseToTriangle(tri, p, body.ClickImpulseStrength);
            body.StartDragTriangle(tri, p);
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (body == null || !body.IsDragging) return;
            body.UpdateDrag(new Vec2(e.X, e.Y));
        }

        private void OnMouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || body == null) return;
            body.EndDrag();
        }
        
        /// <summary> returns the inner play area rectangle, so the space inside the red frame </summary>
        private SKRect InnerBoundsRectangle()
        {
            float pad = FrameStrokeWidth + InnerPadding;
            return new SKRect(pad, pad, canvasControl.Width - pad, canvasControl.Height - pad);
        }
        
        private Vec2 WindowScreenPosition()
        {
            var p = this.DesktopLocation;
            return new Vec2(p.X, p.Y);
        }
    }
}
*/
// END OF OLD CODE

// Sources: https://matthias-research.github.io/pages/tenMinutePhysics/index.html (and all sub-pages/ videos/ github-repositories)
//          https://www.youtube.com/watch?v=SIyKD0Qx2Ak / https://github.com/TheSandwichCoder/SoftBodySimulation-Rust / https://github.com/TheSandwichCoder/SoftBodyTetris
//          some ideas from https://www.youtube.com/watch?v=3OmkehAJoyo
//          barycentric coordinates - https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates.html and https://math.stackexchange.com/questions/1887215/fast-way-of-computing-barycentric-coordinates-explained?
//          StackOverflow for smaller fixes

// The new Code:



namespace GooseWiggle
{
    public static class Program
    {
        [STAThread]
        public static void Main()
        {
            ApplicationConfiguration.Initialize();
            Application.Run(new GooseForm());
        }
    }

    public sealed class GooseForm : Form
    {
        // UI
        private readonly SKControl _canvas;
        private readonly Timer _frameTimer;

        // Simulation
        private SoftBody _body = null!;
        private DateTime _lastFrameTime;
        private Vec2 _prevWinPos;
        private Vec2 _prevWinVel;

        private const int FrameStrokeWidth = 20;
        private const int InnerPadding = 0;

        public GooseForm()
        {
            Text = "Goose Triangle-Softbody 2D – Wiggle Physics";
            ClientSize = new System.Drawing.Size(820, 820);

            _canvas = new SKControl { Dock = DockStyle.Fill };
            _canvas.PaintSurface += OnPaintSurface;
            _canvas.MouseDown += OnMouseDown;
            _canvas.MouseMove += OnMouseMove;
            _canvas.MouseUp += OnMouseUp;
            _canvas.MouseLeave += (_, __) => _body?.EndDrag();
            Controls.Add(_canvas);

            BuildBody();

            _lastFrameTime = DateTime.Now;
            _prevWinPos = WindowScreenPosition();

            _frameTimer = new Timer { Interval = 16 };
            _frameTimer.Tick += (_, __) => TickFrame();
            _frameTimer.Start();

            Resize += (_, __) => { if (_body != null) _body.Bounds = InnerBoundsRectangle(); };
        }

        private void BuildBody()
        {
            var tris = Goose.Create();
            _body = SoftBody.FromTriangles(tris, InnerBoundsRectangle());
        }

        private SKRect InnerBoundsRectangle()
        {
            float pad = FrameStrokeWidth + InnerPadding;
            return new SKRect(pad, pad, _canvas.Width - pad, _canvas.Height - pad);
        }

        private void TickFrame()
        {
            var now = DateTime.Now;
            float dt = (float)(now - _lastFrameTime).TotalSeconds;
            if (dt <= 0f || dt > 0.25f) dt = 1f / 60f;
            _lastFrameTime = now;

            // window movement → acceleration “shake”
            var winPos = WindowScreenPosition();
            var winVel = (winPos - _prevWinPos) / Math.Max(dt, 1e-6f);
            var winAcc = (winVel - _prevWinVel) / Math.Max(dt, 1e-6f);
            _prevWinPos = winPos;
            _prevWinVel = winVel;

            _body.Bounds = InnerBoundsRectangle();
            _body.SimulationSteps(dt, winAcc);

            _canvas.Invalidate();
        }

        private void OnPaintSurface(object? sender, SKPaintSurfaceEventArgs e)
        {
            var canvas = e.Surface.Canvas;
            canvas.Clear(SKColors.White);

            using (var border = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStrokeWidth, IsAntialias = true })
                canvas.DrawRect(0, 0, e.Info.Width, e.Info.Height, border);

            _body.DrawMesh(canvas);
        }

        private void OnMouseDown(object? sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || _body == null) return;
            var p = new Vec2(e.X, e.Y);

            // Try precise hit via barycentric; fallback to nearest centroid
            if (!TryPickTriangleAndWeights(p, out int triIndex, out float u, out float v, out float w))
            {
                triIndex = NearestTriangleByCentroid(p, out u, out v, out w);
                if (triIndex < 0) return;
            }

            // Small radial impulse + start drag at barycentric weights
            _body.ApplyClickImpulse(p);
            _body.StartDrag(triIndex, u, v, w, p);
        }

        private void OnMouseMove(object? sender, MouseEventArgs e)
        {
            if (_body == null || !_body.IsDragging) return;
            _body.UpdateDrag(new Vec2(e.X, e.Y));
        }

        private void OnMouseUp(object? sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || _body == null) return;
            _body.EndDrag();
        }

        // ---------- Picking helpers ----------

        private bool TryPickTriangleAndWeights(Vec2 p, out int triIndex, out float u, out float v, out float w)
        {
            const float EPS = -1e-3f;
            for (int i = 0; i < _body.Triangles.Count; i++)
            {
                var t = _body.Triangles[i];
                var A = _body.Vertices[t.A].CurrentPosition;
                var B = _body.Vertices[t.B].CurrentPosition;
                var C = _body.Vertices[t.C].CurrentPosition;

                if (TryBarycentric(p, A, B, C, out u, out v, out w))
                {
                    if (u >= EPS && v >= EPS && w >= EPS)
                    {
                        triIndex = i;
                        return true;
                    }
                }
            }
            triIndex = -1; u = v = w = 0;
            return false;
        }

        private int NearestTriangleByCentroid(Vec2 p, out float u, out float v, out float w)
        {
            int best = -1;
            float bestDist = float.MaxValue;
            u = v = w = 0;

            for (int i = 0; i < _body.Triangles.Count; i++)
            {
                var t = _body.Triangles[i];
                var A = _body.Vertices[t.A].CurrentPosition;
                var B = _body.Vertices[t.B].CurrentPosition;
                var C = _body.Vertices[t.C].CurrentPosition;

                var centroid = new Vec2((A.X + B.X + C.X) / 3f, (A.Y + B.Y + C.Y) / 3f);
                float d2 = (centroid - p).LengthSq;
                if (d2 < bestDist)
                {
                    bestDist = d2;
                    best = i;
                    // still compute barycentric for stable drag weights
                    TryBarycentric(p, A, B, C, out u, out v, out w);
                }
            }
            return best;
        }

        private static bool TryBarycentric(in Vec2 P, in Vec2 A, in Vec2 B, in Vec2 C, out float u, out float v, out float w)
        {
            float denom = (B.Y - C.Y) * (A.X - C.X) + (C.X - B.X) * (A.Y - C.Y);
            if (MathF.Abs(denom) < 1e-6f) { u = v = w = 0; return false; }
            u = ((B.Y - C.Y) * (P.X - C.X) + (C.X - B.X) * (P.Y - C.Y)) / denom;
            v = ((C.Y - A.Y) * (P.X - C.X) + (A.X - C.X) * (P.Y - C.Y)) / denom;
            w = 1f - u - v;
            return true;
        }

        private Vec2 WindowScreenPosition()
        {
            var p = DesktopLocation;
            return new Vec2(p.X, p.Y);
        }
    }
}
