using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
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
        // CLI options
        private string? _triPath;
        private readonly bool _openLatest;
        private readonly bool _fallbackGoose;
        private readonly Size? _explicitSize; // --size=WxH

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

        public GooseForm(string[]? args = null)
        {
            // parse flags
            string? tri = null; bool openLatest = false; bool fallbackGoose = false; Size? explicitSz = null;
            if (args != null)
            {
                foreach (var a in args)
                {
                    if (a.StartsWith("--tri=")) tri = a.Substring("--tri=".Length).Trim().Trim('\"', ' ');
                    else if (a == "--open-latest") openLatest = true;
                    else if (a == "--fallback=goose") fallbackGoose = true;
                    else if (a.StartsWith("--size="))
                    {
                        var s = a.Substring("--size=".Length);
                        var parts = s.Split('x', 'X');
                        if (parts.Length == 2 && int.TryParse(parts[0], out int w) && int.TryParse(parts[1], out int h))
                            explicitSz = new Size(Math.Max(200, w), Math.Max(200, h));
                    }
                }
            }
            _triPath = tri; _openLatest = openLatest; _fallbackGoose = fallbackGoose; _explicitSize = explicitSz;

            Text = "Goose Triangle-Softbody 2D – Wiggle Physics";
            ClientSize = new Size(820, 820); // temp; adjusted after loading

            _canvas = new SKControl { Dock = DockStyle.Fill };
            _canvas.PaintSurface += OnPaintSurface;
            _canvas.MouseDown += OnMouseDown;
            _canvas.MouseMove += OnMouseMove;
            _canvas.MouseUp += OnMouseUp;
            _canvas.MouseLeave += (_, __) => _body?.EndDrag();
            Controls.Add(_canvas);

            BuildBodyAndSizeWindow();

            _lastFrameTime = DateTime.Now;
            _prevWinPos = WindowScreenPosition();

            _frameTimer = new Timer { Interval = 16 };
            _frameTimer.Tick += (_, __) => TickFrame();
            _frameTimer.Start();
        }

        private void BuildBodyAndSizeWindow()
        {
            // 1) Load triangles (or goose preset)
            List<Triangle>? tris = null;
            try
            {
                if (!string.IsNullOrWhiteSpace(_triPath) && System.IO.File.Exists(_triPath))
                {
                    tris = TriangleIo.Load(_triPath);
                }
                else if (_openLatest)
                {
                    var imagesDir = System.IO.Path.GetFullPath(
                        System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "Images"));
                    if (System.IO.Directory.Exists(imagesDir))
                    {
                        var files = System.IO.Directory.GetFiles(imagesDir, "*.json");
                        if (files.Length > 0)
                        {
                            Array.Sort(files, (a, b) => System.IO.File.GetLastWriteTime(b).CompareTo(System.IO.File.GetLastWriteTime(a)));
                            _triPath = files[0];
                            tris = TriangleIo.Load(_triPath);
                        }
                    }
                }
            }
            catch { /* ignore */ }

            if (tris == null || tris.Count == 0) tris = Goose.Create();

            // 2) Compute source bbox (in JSON/image coordinates)
            GetBounds(tris, out float minX, out float minY, out float maxX, out float maxY);
            float srcW = MathF.Max(1, maxX - minX);
            float srcH = MathF.Max(1, maxY - minY);
            float aspect = srcW / srcH;

            // 3) Choose inner (content) size by aspect or explicit window size
            Size inner;
            if (_explicitSize.HasValue)
            {
                var full = _explicitSize.Value;
                inner = new Size(Math.Max(200, full.Width - 2 * FrameStrokeWidth),
                                 Math.Max(200, full.Height - 2 * FrameStrokeWidth));
            }
            else
            {
                int longSide = 900;
                inner = (aspect >= 1f)
                    ? new Size(longSide, Math.Max(300, (int)(longSide / aspect)))
                    : new Size(Math.Max(300, (int)(longSide * aspect)), longSide);
            }

            // 4) Set window client size = inner + frame
            ClientSize = new Size(inner.Width + 2 * FrameStrokeWidth, inner.Height + 2 * FrameStrokeWidth);

            // 5) Fit triangles into the inner rectangle (scale + center)
            var innerRect = InnerBoundsRectangle();
            var fitted = FitTrianglesToRect(tris, innerRect);

            // 6) Build soft body from the **fitted** triangles
            _body = SoftBody.FromTriangles(fitted, innerRect);
        }

        private static void GetBounds(List<Triangle> tris, out float minX, out float minY, out float maxX, out float maxY)
        {
            minX = minY = float.MaxValue; maxX = maxY = float.MinValue;
            foreach (var t in tris)
            {
                minX = MathF.Min(minX, MathF.Min(t.A.X, MathF.Min(t.B.X, t.C.X)));
                minY = MathF.Min(minY, MathF.Min(t.A.Y, MathF.Min(t.B.Y, t.C.Y)));
                maxX = MathF.Max(maxX, MathF.Max(t.A.X, MathF.Max(t.B.X, t.C.X)));
                maxY = MathF.Max(maxY, MathF.Max(t.A.Y, MathF.Max(t.B.Y, t.C.Y)));
            }
        }

        private static List<Triangle> FitTrianglesToRect(List<Triangle> src, SKRect target)
        {
            // compute bbox
            GetBounds(src, out float minX, out float minY, out float maxX, out float maxY);
            float w = MathF.Max(1, maxX - minX);
            float h = MathF.Max(1, maxY - minY);

            // uniform scale to fit
            float sx = target.Width / w;
            float sy = target.Height / h;
            float s = MathF.Min(sx, sy);

            // center inside target
            float contentW = s * w;
            float contentH = s * h;
            float offX = target.Left + (target.Width  - contentW) * 0.5f - s * minX;
            float offY = target.Top  + (target.Height - contentH) * 0.5f - s * minY;

            static SKPoint Xform(SKPoint p, float s, float ox, float oy)
                => new SKPoint(p.X * s + ox, p.Y * s + oy);

            var dst = new List<Triangle>(src.Count);
            foreach (var t in src)
            {
                dst.Add(new Triangle(
                    Xform(t.A, s, offX, offY),
                    Xform(t.B, s, offX, offY),
                    Xform(t.C, s, offX, offY),
                    t.FillColor
                ));
            }
            return dst;
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

            var winPos = WindowScreenPosition();
            var winVel = (winPos - _prevWinPos) / Math.Max(dt, 1e-6f);
            var winAcc = (winVel - _prevWinVel) / Math.Max(dt, 1e-6f);
            _prevWinPos = winPos; _prevWinVel = winVel;

            _body.Bounds = InnerBoundsRectangle();
            _body.SimulationSteps(dt, winAcc);

            _canvas.Invalidate();
        }

        private void OnPaintSurface(object? sender, SKPaintSurfaceEventArgs e)
        {
            var canvas = e.Surface.Canvas;
            canvas.Clear(SKColors.DarkGray);

            using (var border = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStrokeWidth, IsAntialias = true })
                canvas.DrawRect(0, 0, e.Info.Width, e.Info.Height, border);

            // single bottom baseline (no fill)
            float lowestY = (_body != null && _body.Vertices.Count > 0)
                ? _body.Vertices.Max(v => v.CurrentPosition.Y)
                : e.Info.Height - FrameStrokeWidth;

            using (var baseline = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStrokeWidth, IsAntialias = true })
                canvas.DrawLine(0, lowestY, e.Info.Width, lowestY, baseline);

            _body.DrawMesh(canvas);
        }

        // ----- Mouse: pick triangle + barycentric weights, then StartDrag(tri, u, v, w, p) -----
        private void OnMouseDown(object? sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || _body == null) return;
            var p = new Vec2(e.X, e.Y);

            if (!TryPickTriangleAndWeights(p, out int triIndex, out float u, out float v, out float w))
            {
                triIndex = NearestTriangleByCentroid(p, out u, out v, out w);
                if (triIndex < 0) return;
            }

            try { _body.ApplyClickImpulse(p); } catch { /* optional */ }
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

        // ---- Picking helpers ----
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
                    if (u >= EPS && v >= EPS && w >= EPS) { triIndex = i; return true; }
                }
            }
            triIndex = -1; u = v = w = 0;
            return false;
        }

        private int NearestTriangleByCentroid(Vec2 p, out float u, out float v, out float w)
        {
            int best = -1; float bestDist = float.MaxValue; u = v = w = 0;
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
                    bestDist = d2; best = i;
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
            var p = DesktopLocation;
            return new Vec2(p.X, p.Y);
        }
    }
}
