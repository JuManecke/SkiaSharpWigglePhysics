using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.IO;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using Wiggle.Core;
using Wiggle.Core.Presets;

namespace GooseWiggle
{
    public static class Program
    {
        [STAThread]
        public static void Main(string[] args)
        {
            ApplicationConfiguration.Initialize();
            Application.Run(new GooseForm(args));
        }
    }

    public sealed class GooseForm : Form
    {
        private string? _triPath;
        private readonly Size? _explicitSize;

        private readonly SKControl _canvas;
        private readonly Timer _frameTimer;

        private SoftBody _body = null!;
        private DateTime _lastFrameTime;
        private Vec2 _prevWinPos;
        private Vec2 _prevWinVel;

        private const int FrameStrokeWidth = 20;
        private const int InnerPadding = 0;

        public GooseForm(string[]? args = null)
        {
            Size? explicitSz = null;
            if (args != null)
            {
                for (int i = 0; i < args.Length; i++)
                {
                    string a = args[i];
                    if (a.StartsWith("--size="))
                    {
                        string s = a.Substring("--size=".Length);
                        string[] parts = s.Split(new char[] { 'x', 'X' }, StringSplitOptions.RemoveEmptyEntries);
                        int w, h;
                        if (parts.Length == 2 && int.TryParse(parts[0], out w) && int.TryParse(parts[1], out h))
                        {
                            if (w < 200) w = 200;
                            if (h < 200) h = 200;
                            explicitSz = new Size(w, h);
                        }
                    }
                }
            }

            _triPath = GetTrianglesPath(args);
            _explicitSize = explicitSz;

            Text = "Goose Triangle-Softbody 2D – Wiggle Physics";
            ClientSize = new Size(820, 820);

            _canvas = new SKControl();
            _canvas.Dock = DockStyle.Fill;
            _canvas.PaintSurface += Canvas_PaintSurface;
            _canvas.MouseDown += Canvas_MouseDown;
            _canvas.MouseMove += Canvas_MouseMove;
            _canvas.MouseUp += Canvas_MouseUp;
            _canvas.MouseLeave += Canvas_MouseLeave;
            Controls.Add(_canvas);

            BuildBodyAndSizeWindow();

            _lastFrameTime = DateTime.Now;
            _prevWinPos = WindowScreenPosition();

            _frameTimer = new Timer();
            _frameTimer.Interval = 16;
            _frameTimer.Tick += FrameTimer_Tick;
            _frameTimer.Start();
        }

        private void BuildBodyAndSizeWindow()
        {
            List<Triangle>? tris = null;
            try
            {
                if (!string.IsNullOrWhiteSpace(_triPath) && File.Exists(_triPath))
                {
                    tris = TriangleIo.Load(_triPath);
                }
            }
            catch { }

            if (tris == null || tris.Count == 0)
            {
                tris = Goose.Create();
            }
            
            float minX, minY, maxX, maxY;
            GetBounds(tris, out minX, out minY, out maxX, out maxY);
            float srcW = MathF.Max(1f, maxX - minX);
            float srcH = MathF.Max(1f, maxY - minY);
            float aspect = srcW / srcH;

            // choose inner size
            Size inner;
            if (_explicitSize.HasValue)
            {
                Size full = _explicitSize.Value;
                inner = new Size(
                    Math.Max(200, full.Width - 2 * FrameStrokeWidth),
                    Math.Max(200, full.Height - 2 * FrameStrokeWidth)
                );
            }
            else
            {
                int longSide = 900;
                if (aspect >= 1f)
                {
                    inner = new Size(longSide, Math.Max(300, (int)(longSide / aspect)));
                }
                else
                {
                    inner = new Size(Math.Max(300, (int)(longSide * aspect)), longSide);
                }
            }

            // window size = inner + border
            ClientSize = new Size(inner.Width + 2 * FrameStrokeWidth, inner.Height + 2 * FrameStrokeWidth);

            // fit triangles into inner rect
            SKRect innerRect = InnerBoundsRectangle();
            List<Triangle> fitted = FitTrianglesToRect(tris, innerRect);

            _body = SoftBody.FromTriangles(fitted, innerRect);
        }

        private static void GetBounds(List<Triangle> tris, out float minX, out float minY, out float maxX, out float maxY)
        {
            minX = float.MaxValue;
            minY = float.MaxValue;
            maxX = float.MinValue;
            maxY = float.MinValue;

            for (int i = 0; i < tris.Count; i++)
            {
                Triangle t = tris[i];

                if (t.A.X < minX) minX = t.A.X;
                if (t.B.X < minX) minX = t.B.X;
                if (t.C.X < minX) minX = t.C.X;

                if (t.A.Y < minY) minY = t.A.Y;
                if (t.B.Y < minY) minY = t.B.Y;
                if (t.C.Y < minY) minY = t.C.Y;

                if (t.A.X > maxX) maxX = t.A.X;
                if (t.B.X > maxX) maxX = t.B.X;
                if (t.C.X > maxX) maxX = t.C.X;

                if (t.A.Y > maxY) maxY = t.A.Y;
                if (t.B.Y > maxY) maxY = t.B.Y;
                if (t.C.Y > maxY) maxY = t.C.Y;
            }

            if (minX == float.MaxValue)
            {
                minX = minY = 0f;
                maxX = maxY = 1f;
            }
        }

        private static List<Triangle> FitTrianglesToRect(List<Triangle> src, SKRect target)
        {
            float minX, minY, maxX, maxY;
            GetBounds(src, out minX, out minY, out maxX, out maxY);

            float w = MathF.Max(1f, maxX - minX);
            float h = MathF.Max(1f, maxY - minY);

            float sx = target.Width / w;
            float sy = target.Height / h;
            float s = sx < sy ? sx : sy;

            float contentW = s * w;
            float contentH = s * h;

            float offX = target.Left + (target.Width - contentW) * 0.5f - s * minX;
            float offY = target.Top + (target.Height - contentH) * 0.5f - s * minY;

            List<Triangle> dst = new List<Triangle>(src.Count);
            for (int i = 0; i < src.Count; i++)
            {
                Triangle t = src[i];

                SKPoint A = new SKPoint(t.A.X * s + offX, t.A.Y * s + offY);
                SKPoint B = new SKPoint(t.B.X * s + offX, t.B.Y * s + offY);
                SKPoint C = new SKPoint(t.C.X * s + offX, t.C.Y * s + offY);

                dst.Add(new Triangle(A, B, C, t.FillColor));
            }
            return dst;
        }

        private SKRect InnerBoundsRectangle()
        {
            float pad = FrameStrokeWidth + InnerPadding;
            return new SKRect(pad, pad, _canvas.Width - pad, _canvas.Height - pad);
        }

        private void FrameTimer_Tick(object? sender, EventArgs e)
        {
            TickFrame();
        }

        private void TickFrame()
        {
            DateTime now = DateTime.Now;
            float dt = (float)(now - _lastFrameTime).TotalSeconds;
            if (dt <= 0f || dt > 0.25f) dt = 1f / 60f;
            _lastFrameTime = now;

            Vec2 winPos = WindowScreenPosition();
            Vec2 winVel = (winPos - _prevWinPos) / Math.Max(dt, 1e-6f);
            Vec2 winAcc = (winVel - _prevWinVel) / Math.Max(dt, 1e-6f);
            _prevWinPos = winPos;
            _prevWinVel = winVel;

            _body.Bounds = InnerBoundsRectangle();
            _body.SimulationSteps(dt, winAcc);

            _canvas.Invalidate();
        }

        private void Canvas_PaintSurface(object? sender, SKPaintSurfaceEventArgs e)
        {
            SKCanvas canvas = e.Surface.Canvas;
            canvas.Clear(SKColors.DarkGray);

            using (SKPaint border = new SKPaint())
            {
                border.Color = SKColors.Red;
                border.IsStroke = true;
                border.StrokeWidth = FrameStrokeWidth;
                border.IsAntialias = true;
                canvas.DrawRect(0, 0, e.Info.Width, e.Info.Height, border);
            }

            float lowestY = FindLowestVertexY(_body, e.Info.Height);

            using (SKPaint baseline = new SKPaint())
            {
                baseline.Color = SKColors.Red;
                baseline.IsStroke = true;
                baseline.StrokeWidth = FrameStrokeWidth;
                baseline.IsAntialias = true;
                canvas.DrawLine(0, lowestY, e.Info.Width, lowestY, baseline);
            }

            _body.DrawMesh(canvas);
        }

        private static float FindLowestVertexY(SoftBody body, int fallback)
        {
            if (body == null || body.Vertices == null || body.Vertices.Count == 0)
                return fallback - FrameStrokeWidth;

            float y = body.Vertices[0].CurrentPosition.Y;
            for (int i = 1; i < body.Vertices.Count; i++)
            {
                float vy = body.Vertices[i].CurrentPosition.Y;
                if (vy > y) y = vy;
            }
            return y;
        }

        private void Canvas_MouseDown(object? sender, MouseEventArgs e)
        {
            if (_body == null || e.Button != MouseButtons.Left) return;

            Vec2 p = new Vec2(e.X, e.Y);

            int triIndex;
            float u, v, w;

            if (!TryPickTriangleAndWeights(p, out triIndex, out u, out v, out w))
            {
                triIndex = NearestTriangleByCentroid(p, out u, out v, out w);
                if (triIndex < 0) return;
            }

            try { _body.ApplyClickImpulse(p); } catch { }
            _body.StartDrag(triIndex, u, v, w, p);
        }

        private void Canvas_MouseMove(object? sender, MouseEventArgs e)
        {
            if (_body == null || !_body.IsDragging) return;
            _body.UpdateDrag(new Vec2(e.X, e.Y));
        }

        private void Canvas_MouseUp(object? sender, MouseEventArgs e)
        {
            if (_body == null || e.Button != MouseButtons.Left) return;
            _body.EndDrag();
        }

        private void Canvas_MouseLeave(object? sender, EventArgs e)
        {
            if (_body != null) _body.EndDrag();
        }

        private bool TryPickTriangleAndWeights(Vec2 p, out int triIndex, out float u, out float v, out float w)
        {
            const float EPS = -1e-3f;

            for (int i = 0; i < _body.Triangles.Count; i++)
            {
                var t = _body.Triangles[i];
                Vec2 A = _body.Vertices[t.A].CurrentPosition;
                Vec2 B = _body.Vertices[t.B].CurrentPosition;
                Vec2 C = _body.Vertices[t.C].CurrentPosition;

                if (TryBarycentric(p, A, B, C, out u, out v, out w))
                {
                    if (u >= EPS && v >= EPS && w >= EPS)
                    {
                        triIndex = i;
                        return true;
                    }
                }
            }

            triIndex = -1;
            u = v = w = 0f;
            return false;
        }

        private int NearestTriangleByCentroid(Vec2 p, out float u, out float v, out float w)
        {
            int best = -1;
            float bestDist = float.MaxValue;
            u = v = w = 0f;

            for (int i = 0; i < _body.Triangles.Count; i++)
            {
                var t = _body.Triangles[i];
                Vec2 A = _body.Vertices[t.A].CurrentPosition;
                Vec2 B = _body.Vertices[t.B].CurrentPosition;
                Vec2 C = _body.Vertices[t.C].CurrentPosition;

                Vec2 centroid = new Vec2((A.X + B.X + C.X) / 3f, (A.Y + B.Y + C.Y) / 3f);
                float d2 = (centroid - p).LengthSq;
                if (d2 < bestDist)
                {
                    bestDist = d2;
                    best = i;
                    TryBarycentric(p, A, B, C, out u, out v, out w);
                }
            }

            return best;
        }

        private static bool TryBarycentric(in Vec2 P, in Vec2 A, in Vec2 B, in Vec2 C,
                                           out float u, out float v, out float w)
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
            Point p = DesktopLocation;
            return new Vec2(p.X, p.Y);
        }
        

        private static string? GetTrianglesPath(string[]? args)
        {
            if (args == null || args.Length == 0)
                return FindLatestJson();

            string? triArg = null;
            bool openLatest = false;

            for (int i = 0; i < args.Length; i++)
            {
                string a = args[i];
                if (a.StartsWith("--tri=")) triArg = a.Substring(6).Trim('"');
                else if (a == "--tri") triArg = NextArg(args, i);
                else if (a == "--open-latest") openLatest = true;
            }

            if (!string.IsNullOrWhiteSpace(triArg))
            {
                string? p = ResolveTriPath(triArg);
                if (p != null) return p;
                return null;
            }

            if (openLatest) return FindLatestJson();
            return FindLatestJson();
        }

        private static string? NextArg(string[] args, int index)
        {
            if (index + 1 < args.Length) return args[index + 1];
            return null;
        }

        private static string? ResolveTriPath(string raw)
        {
            if (string.IsNullOrWhiteSpace(raw)) return null;

            string root = FindRepoRoot();
            string baseDir = AppContext.BaseDirectory;

            string[] candidates = new string[]
            {
                raw,
                SafeFullPath(raw),
                Path.Combine(baseDir, raw),
                Path.Combine(baseDir, "Images", raw),
                Path.Combine(root, raw),
                Path.Combine(root, "Images", raw)
            };

            for (int i = 0; i < candidates.Length; i++)
            {
                string c = candidates[i];
                if (!string.IsNullOrEmpty(c) && File.Exists(c))
                {
                    return SafeFullPath(c);
                }
            }

            return null;
        }

        private static string? FindLatestJson()
        {
            string root = FindRepoRoot();
            string imagesDir = Path.Combine(root, "Images");
            if (!Directory.Exists(imagesDir)) return null;

            string[] files = Directory.GetFiles(imagesDir, "*.json", SearchOption.AllDirectories);
            if (files.Length == 0) return null;

            string newest = files[0];
            DateTime newestTime = File.GetLastWriteTimeUtc(newest);

            for (int i = 1; i < files.Length; i++)
            {
                string f = files[i];
                DateTime t = File.GetLastWriteTimeUtc(f);
                if (t > newestTime)
                {
                    newestTime = t;
                    newest = f;
                }
            }

            return SafeFullPath(newest);
        }

        private static string FindRepoRoot()
        {
            string[] starts = new string[]
            {
                Directory.GetCurrentDirectory(),
                AppContext.BaseDirectory
            };

            for (int si = 0; si < starts.Length; si++)
            {
                DirectoryInfo dir = new DirectoryInfo(starts[si]);
                int depth = 0;
                while (dir != null && depth < 10)
                {
                    string sln = Path.Combine(dir.FullName, "SkiaSharpWigglePhysics.sln");
                    if (File.Exists(sln)) return dir.FullName;
                    dir = dir.Parent!;
                    depth++;
                }
            }
            return Directory.GetCurrentDirectory();
        }

        private static string SafeFullPath(string p)
        {
            try { return Path.GetFullPath(p); } catch { return p; }
        }
    }
}
