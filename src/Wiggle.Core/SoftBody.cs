using System;
using System.Collections.Generic;
using System.Linq;
using SkiaSharp;

namespace Wiggle.Core
{
    // General idea:      A softbody mesh made of triangles, in the shape of a goose, that jiggles when the window moves or when poked/dragged with the mouse.
    // General behaviour: The mesh jiggles when the window moves, the triangles can be clicked to poke them or can be dragged around softly.
    //                    The bottom row of the mesh is pinned to keep the body upright and the triangles should not flip or collapse.
    // General structure: The mesh is made of vertices, edges, and triangles. Every vertex is treated as a particle and uses verlet integration.
    //                    The verlet integration is done in the SoftBody class, which also handles the physics, movement and drawing of the mesh.
    //                    The mesh keeps its general shape with distance constraints on the edges, and a few long-range edges to prevent collapse or overstretching.
    //                    A no-flip constraint keeps triangles from inverting or collapsing too thin and a tether pulls the mesh back to its default position when idle.
    // How to use it?:    Move the window to make the goose jiggle, click triangles to poke them, and drag triangles to move them around.
    //                    Have fun! Honk! Honk! Honk!
    //
    // Basis:             Vector2 class with + - * / length normal dot 90°-perp is needed for the physics
    //                    Triangle class to hold the body with its color
    //                    Vertex class to hold the points of the mesh
    //                    Edge class to hold the edges of the mesh
    //                    TriangleIndex class to hold the triangle indices and orientation references
    //                    SoftBody class to hold the mesh, run the physics, and draw the body
    //                    GooseForm class to hold the window, canvas, and input handling
    //
    // At start:          The window is created with a Skia canvas inside it.
    //                    The goose shape is defined as a list of triangles with their colors.
    //
    // The mesh:          The SoftBody is created from the triangles, building the vertices and edges, and pinning the bottom row.
    //                    Vertices of the triangles that are identical in position are shared to avoid duplicates.
    //                    Triangles are stored with their vertex indices and orientation references for no-flip constraints.
    //                    Edges are created for each triangle edge as distance constraints.
    //                    Long-range edges are added to keep the overall shape and reduce large-scale sagging.
    //
    // At runtime:        The window movement gives acceleration into the sim to create a jiggle.
    //                    The canvas redraws on a timer, running the physics step and drawing the mesh.
    //                    The physics uses verlet integration, frame collisions and simple self-collision.
    //                    Distance constraints keep edge lengths, no-flip keeps orientation/thickness, tether returns to rest when idle.
    //                    Mouse input lets me click to poke and drag triangles softly.
    // Sources: https://matthias-research.github.io/pages/tenMinutePhysics/index.html (and all sub-pages/ videos/ github-repositories)
    //          https://www.youtube.com/watch?v=SIyKD0Qx2Ak / https://github.com/TheSandwichCoder/SoftBodySimulation-Rust / https://github.com/TheSandwichCoder/SoftBodyTetris
    //          some ideas from https://www.youtube.com/watch?v=3OmkehAJoyo
    //          barycentric coordinates - https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates.html and https://math.stackexchange.com/questions/1887215/fast-way-of-computing-barycentric-coordinates-explained?
    //          StackOverflow for smaller fixes

    public class SoftBody
    {
        public List<Vertex> Vertices = new();
        public List<Edge> Edges = new();
        public List<TriangleIndex> Triangles = new();

        // Tunables
        public float VelocityDamping = 0.990f;
        public float EdgeStiffness = 0.85f;
        public int   UpdateAtWhichFPS = 25;
        public float WallBounce = 0.8f;
        public float DefaultPositionPullStrength = 0.035f;
        public float WindowShakeStrength = 1.6f;

        public float LongRangeStiffness = 0.18f;
        public int   LongRangeMaxNeighbors = 2;
        public float LongRangeMaxDistance = 220f;

        public float ClickImpulseStrength = 24f;
        public bool  IsDragging = false;
        public int   DragTriangleIndex = -1;
        public float DragWeightU, DragWeightV, DragWeightW;
        public Vec2  DragTarget;
        public float DragStiffness = 0.6f;

        public SKRect Bounds;

        public static SoftBody FromTriangles(List<Triangle> input, SKRect bounds)
        {
            var body = new SoftBody { Bounds = bounds };
            var indexOf = new Dictionary<(int X, int Y), int>();

            int GetOrCreateVertex(SKPoint p)
            {
                var key = ((int)MathF.Round(p.X), (int)MathF.Round(p.Y));
                if (!indexOf.TryGetValue(key, out var idx))
                {
                    idx = body.Vertices.Count;
                    body.Vertices.Add(new Vertex
                    {
                        CurrentPosition  = new Vec2(key.Item1, key.Item2),
                        PreviousPosition = new Vec2(key.Item1, key.Item2),
                        DefaultPosition  = new Vec2(key.Item1, key.Item2),
                        Radius = 5f
                    });
                    indexOf[key] = idx;
                }
                return idx;
            }

            foreach (var t in input)
            {
                int a = GetOrCreateVertex(t.A);
                int b = GetOrCreateVertex(t.B);
                int c = GetOrCreateVertex(t.C);

                var A0 = body.Vertices[a].DefaultPosition;
                var B0 = body.Vertices[b].DefaultPosition;
                var C0 = body.Vertices[c].DefaultPosition;

                float hC = GetHeight(C0, A0, B0);
                float hA = GetHeight(A0, B0, C0);
                float hB = GetHeight(B0, C0, A0);

                body.Triangles.Add(new TriangleIndex
                {
                    A = a, B = b, C = c,
                    Fill = t.FillColor,
                    DefaultHeight_C_AB = hC,
                    DefaultHeight_A_BC = hA,
                    DefaultHeight_B_CA = hB,
                    MinHeightFraction = 0.35f
                });
            }

            var edgeSet = new HashSet<(int, int)>();
            void AddEdgeIfNew(int i, int j, float stiffness)
            {
                if (i == j) return;
                if (i > j) (i, j) = (j, i);
                if (edgeSet.Add((i, j)))
                {
                    float L = (body.Vertices[j].CurrentPosition - body.Vertices[i].CurrentPosition).Length;
                    body.Edges.Add(new Edge { I = i, J = j, DefaultLength = L, Stiffness = stiffness });
                }
            }
            foreach (var tr in body.Triangles)
            {
                AddEdgeIfNew(tr.A, tr.B, body.EdgeStiffness);
                AddEdgeIfNew(tr.B, tr.C, body.EdgeStiffness);
                AddEdgeIfNew(tr.C, tr.A, body.EdgeStiffness);
            }

            // Long range edges
            for (int i = 0; i < body.Vertices.Count; i++)
            {
                var pi = body.Vertices[i].CurrentPosition;
                var candidates = new List<(float d, int j)>();
                for (int j = 0; j < body.Vertices.Count; j++)
                {
                    if (i == j) continue;
                    float d = (body.Vertices[j].CurrentPosition - pi).Length;
                    if (d <= body.LongRangeMaxDistance) candidates.Add((d, j));
                }
                candidates.Sort((x, y) => x.d.CompareTo(y.d));
                int added = 0;
                foreach (var (d, j) in candidates)
                {
                    if (added >= body.LongRangeMaxNeighbors) break;
                    int a = i, b = j; if (a > b) (a, b) = (b, a);
                    if (edgeSet.Contains((a, b))) continue;
                    edgeSet.Add((a, b));
                    body.Edges.Add(new Edge { I = i, J = j, DefaultLength = d, Stiffness = body.LongRangeStiffness });
                    added++;
                }
            }

            // Pin bottom row
            float maxY = body.Vertices.Max(v => v.CurrentPosition.Y);
            for (int i = 0; i < body.Vertices.Count; i++)
            {
                if (MathF.Abs(body.Vertices[i].CurrentPosition.Y - maxY) < 0.5f)
                {
                    var v = body.Vertices[i];
                    v.IsPinned = true;
                    body.Vertices[i] = v;
                }
            }

            return body;
        }

        private static float GetHeight(in Vec2 P, in Vec2 A, in Vec2 B)
        {
            var n = Vec2.PerpCCW(B - A).Normalized();
            return Vec2.Dot(P - A, n);
        }

        public void ApplyClickImpulse(Vec2 clickPoint)
        {
            for (int i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                var dir = v.CurrentPosition - clickPoint;
                float len = dir.Length;
                if (len > 1e-3f)
                {
                    var impulse = (dir / len) * ClickImpulseStrength;
                    v.PreviousPosition -= impulse;
                    Vertices[i] = v;
                }
            }
        }

        public void StartDrag(int triIndex, float wU, float wV, float wW, Vec2 target)
        {
            IsDragging = true;
            DragTriangleIndex = triIndex;
            DragWeightU = wU; DragWeightV = wV; DragWeightW = wW;
            DragTarget = target;
        }
        public void UpdateDrag(Vec2 target) => DragTarget = target;
        public void EndDrag() { IsDragging = false; DragTriangleIndex = -1; }

        private void ApplySoftDrag()
        {
            if (!IsDragging || DragTriangleIndex < 0 || DragTriangleIndex >= Triangles.Count) return;

            var tr = Triangles[DragTriangleIndex];
            var A = Vertices[tr.A].CurrentPosition;
            var B = Vertices[tr.B].CurrentPosition;
            var C = Vertices[tr.C].CurrentPosition;

            Vec2 handle = DragWeightU * A + DragWeightV * B + DragWeightW * C;
            Vec2 error = DragTarget - handle;

            void Pull(int idx, float w)
            {
                if (w <= 0f) return;
                var v = Vertices[idx];
                if (v.IsPinned) return;
                var move = error * (DragStiffness * w);
                v.CurrentPosition += move;
                v.PreviousPosition += move;
                Vertices[idx] = v;
            }

            Pull(tr.A, DragWeightU);
            Pull(tr.B, DragWeightV);
            Pull(tr.C, DragWeightW);
        }

        public void SimulationSteps(float dt, Vec2 windowAcceleration)
        {
            if (dt <= 0f || dt > 0.1f) dt = 1f / 60f;
            float dt2 = dt * dt;

            // verlet + window jiggle + tether-to-rest
            for (int i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                if (v.IsPinned) continue;

                Vec2 vel = (v.CurrentPosition - v.PreviousPosition) * VelocityDamping;
                Vec2 accel = new Vec2(-windowAcceleration.X, -windowAcceleration.Y) * WindowShakeStrength;
                Vec2 next = v.CurrentPosition + vel + accel * dt2;

                v.PreviousPosition = v.CurrentPosition;
                v.CurrentPosition = next;

                Vec2 toRest = v.DefaultPosition - v.CurrentPosition;
                v.CurrentPosition += toRest * DefaultPositionPullStrength;

                Vertices[i] = v;
            }

            ApplySoftDrag();

            // frame collisions
            for (int i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                if (v.IsPinned) continue;

                Vec2 vel = v.CurrentPosition - v.PreviousPosition;

                if (v.CurrentPosition.X < Bounds.Left)
                {
                    v.CurrentPosition.X = Bounds.Left;
                    v.PreviousPosition.X = v.CurrentPosition.X - vel.X * WallBounce;
                }
                if (v.CurrentPosition.X > Bounds.Right)
                {
                    v.CurrentPosition.X = Bounds.Right;
                    v.PreviousPosition.X = v.CurrentPosition.X - vel.X * WallBounce;
                }
                if (v.CurrentPosition.Y < Bounds.Top)
                {
                    v.CurrentPosition.Y = Bounds.Top;
                    v.PreviousPosition.Y = v.CurrentPosition.Y - vel.Y * WallBounce;
                }
                if (v.CurrentPosition.Y > Bounds.Bottom)
                {
                    v.CurrentPosition.Y = Bounds.Bottom;
                    v.PreviousPosition.Y = v.CurrentPosition.Y - vel.Y * WallBounce;
                }

                Vertices[i] = v;
            }

            // satisfy distance constraints
            for (int it = 0; it < UpdateAtWhichFPS; it++)
            {
                foreach (var e in Edges)
                {
                    var a = Vertices[e.I];
                    var b = Vertices[e.J];
                    Vec2 delta = b.CurrentPosition - a.CurrentPosition;
                    float dist = delta.Length; if (dist < 1e-6f) continue;
                    float diff = (dist - e.DefaultLength) / dist;
                    Vec2 corr = delta * (0.5f * e.Stiffness * diff);
                    if (!a.IsPinned) a.CurrentPosition += corr;
                    if (!b.IsPinned) b.CurrentPosition -= corr;
                    Vertices[e.I] = a; Vertices[e.J] = b;
                }
                ApplySoftDrag();
            }

            // no-flip / minimum thickness on each edge of each triangle
            foreach (var tr in Triangles)
            {
                EnforceSignedHeight(tr.A, tr.B, tr.C, tr.DefaultHeight_C_AB, tr.MinHeightFraction);
                EnforceSignedHeight(tr.B, tr.C, tr.A, tr.DefaultHeight_A_BC, tr.MinHeightFraction);
                EnforceSignedHeight(tr.C, tr.A, tr.B, tr.DefaultHeight_B_CA, tr.MinHeightFraction);
            }
        }

        private void EnforceSignedHeight(int i, int j, int k, float restH, float minFrac)
        {
            var A = Vertices[i];
            var B = Vertices[j];
            var C = Vertices[k];

            Vec2 ab = B.CurrentPosition - A.CurrentPosition;
            float abLen = ab.Length; if (abLen < 1e-6f) return;
            Vec2 n = Vec2.PerpCCW(ab) / abLen;

            float d = Vec2.Dot(C.CurrentPosition - A.CurrentPosition, n);
            float targetSign = MathF.Sign(restH);
            float minMag = MathF.Abs(restH) * minFrac;

            bool wrongSide = MathF.Sign(d) != targetSign;
            bool tooThin = MathF.Abs(d) < minMag;

            if (wrongSide || tooThin)
            {
                float desired = targetSign * MathF.Max(minMag, MathF.Abs(d));
                float corr = desired - d;

                if (!C.IsPinned)
                    C.CurrentPosition += n * corr;
                else
                {
                    if (!A.IsPinned) A.CurrentPosition -= n * (corr * 0.5f);
                    if (!B.IsPinned) B.CurrentPosition -= n * (corr * 0.5f);
                }

                Vertices[i] = A; Vertices[j] = B; Vertices[k] = C;
            }
        }

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
}
