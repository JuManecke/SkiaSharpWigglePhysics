using System;
using System.Collections.Generic;
using SkiaSharp;

namespace Wiggle.Core.Importers
{
    public static class ImageToTriangles
    {
        /// <summary>
        /// Foreground-only triangulation:
        ///   - Any pixel that is NOT near-white is treated as foreground.
        ///   - White-ish background (including anti-aliased edges) is ignored.
        ///
        /// step ........ grid spacing in pixels (smaller = denser mesh)
        /// whiteRgbTol . 0..255 distance to pure white in RGB (higher = more aggressive background removal)
        /// whiteSatMax . HSV saturation threshold for off-whites (0..1)
        /// whiteValMin . HSV value (brightness) threshold for off-whites (0..1)
        /// </summary>
        public static List<Triangle> FromImage(
            SKBitmap src,
            int step = 18,
            int whiteRgbTol = 28,
            float whiteSatMax = 0.20f,
            float whiteValMin = 0.94f)
        {
            if (src == null) throw new ArgumentNullException(nameof(src));
            if (step < 4) step = 4;

            int cols = Math.Max(1, (int)Math.Ceiling(src.Width  / (float)step));
            int rows = Math.Max(1, (int)Math.Ceiling(src.Height / (float)step));
            bool[,] fg = new bool[rows, cols];

            // 1) mark foreground at grid cell centers
            for (int r = 0; r < rows; r++)
            {
                for (int c = 0; c < cols; c++)
                {
                    int cx = Math.Clamp(c * step + step / 2, 0, src.Width  - 1);
                    int cy = Math.Clamp(r * step + step / 2, 0, src.Height - 1);
                    var col = src.GetPixel(cx, cy);
                    fg[r, c] = IsNonWhite(col, whiteRgbTol, whiteSatMax, whiteValMin);
                }
            }

            // 2) keep only the largest connected component (the character)
            KeepLargestComponentInPlace(fg);

            // 3) triangulate foreground cells with **triangle-level filtering**
            var tris = new List<Triangle>();
            for (int r = 0; r < rows - 1; r++)
            {
                for (int c = 0; c < cols - 1; c++)
                {
                    // if the 2×2 block is mostly background, skip quickly
                    int fgCount =
                        (fg[r, c] ? 1 : 0) + (fg[r, c + 1] ? 1 : 0) +
                        (fg[r + 1, c] ? 1 : 0) + (fg[r + 1, c + 1] ? 1 : 0);
                    if (fgCount < 2) continue;

                    float x0 = c * step;                 float y0 = r * step;
                    float x1 = Math.Min(src.Width,  (c + 1) * step);
                    float y1 = Math.Min(src.Height, (r + 1) * step);

                    // tri A: (x0,y0)-(x1,y0)-(x0,y1)
                    var A0 = new SKPoint(x0, y0);
                    var A1 = new SKPoint(x1, y0);
                    var A2 = new SKPoint(x0, y1);
                    if (TriangleIsForeground(src, A0, A1, A2, whiteRgbTol, whiteSatMax, whiteValMin, out var fillA))
                        tris.Add(new Triangle(A0, A1, A2, fillA));

                    // tri B: (x1,y0)-(x1,y1)-(x0,y1)
                    var B0 = new SKPoint(x1, y0);
                    var B1 = new SKPoint(x1, y1);
                    var B2 = new SKPoint(x0, y1);
                    if (TriangleIsForeground(src, B0, B1, B2, whiteRgbTol, whiteSatMax, whiteValMin, out var fillB))
                        tris.Add(new Triangle(B0, B1, B2, fillB));
                }
            }

            return tris;
        }

        // ---- triangle-level foreground test (kills white triangles) ----
        private static bool TriangleIsForeground(
            SKBitmap src, SKPoint p0, SKPoint p1, SKPoint p2,
            int rgbTol, float whiteSatMax, float whiteValMin, out SKColor fill)
        {
            // sample at 3 vertices + centroid
            var c0 = Sample(src, p0);
            var c1 = Sample(src, p1);
            var c2 = Sample(src, p2);
            var pc = new SKPoint((p0.X + p1.X + p2.X) / 3f, (p0.Y + p1.Y + p2.Y) / 3f);
            var cc = Sample(src, pc);

            bool s0 = IsNonWhite(c0, rgbTol, whiteSatMax, whiteValMin);
            bool s1 = IsNonWhite(c1, rgbTol, whiteSatMax, whiteValMin);
            bool s2 = IsNonWhite(c2, rgbTol, whiteSatMax, whiteValMin);
            bool sc = IsNonWhite(cc, rgbTol, whiteSatMax, whiteValMin);

            int votes = (s0 ? 1 : 0) + (s1 ? 1 : 0) + (s2 ? 1 : 0) + (sc ? 1 : 0);
            bool keep = votes >= 3; // require strong majority foreground

            // choose fill color = average of non-white samples (fallback to centroid)
            fill = AverageNonWhite(new[] { c0, c1, c2, cc }, rgbTol, whiteSatMax, whiteValMin, cc);
            return keep;
        }

        private static SKColor Sample(SKBitmap bmp, SKPoint p)
        {
            int x = Math.Clamp((int)MathF.Round(p.X), 0, bmp.Width - 1);
            int y = Math.Clamp((int)MathF.Round(p.Y), 0, bmp.Height - 1);
            return bmp.GetPixel(x, y);
        }

        private static SKColor AverageNonWhite(IEnumerable<SKColor> colors, int rgbTol, float whiteSatMax, float whiteValMin, SKColor fallback)
        {
            long r = 0, g = 0, b = 0; int n = 0;
            foreach (var c in colors)
            {
                if (IsNonWhite(c, rgbTol, whiteSatMax, whiteValMin))
                {
                    r += c.Red; g += c.Green; b += c.Blue; n++;
                }
            }
            if (n == 0) return fallback;
            return new SKColor((byte)(r / n), (byte)(g / n), (byte)(b / n), 255);
        }

        // ---------- color logic: "non-white" = foreground ----------
        private static bool IsNonWhite(SKColor c, int rgbTol, float whiteSatMax, float whiteValMin)
        {
            if (c.Alpha < 16) return false; // transparent → background

            // RGB distance to pure white
            int dr = 255 - c.Red, dg = 255 - c.Green, db = 255 - c.Blue;
            int d2 = dr * dr + dg * dg + db * db;
            bool nearWhiteRgb = d2 <= rgbTol * rgbTol;

            // HSV guard for off-whites/anti-aliased edges
            RgbToHsv(c, out _, out float s, out float v);
            bool whiteishHsv = (s <= whiteSatMax && v >= whiteValMin);

            return !(nearWhiteRgb || whiteishHsv);
        }

        private static void RgbToHsv(SKColor c, out float h, out float s, out float v)
        {
            float r = c.Red / 255f, g = c.Green / 255f, b = c.Blue / 255f;
            float max = MathF.Max(r, MathF.Max(g, b));
            float min = MathF.Min(r, MathF.Min(g, b));
            v = max;
            float d = max - min;
            s = max == 0 ? 0 : d / max;

            if (d == 0) { h = 0; return; }
            if      (max == r) h = 60f * (((g - b) / d + 6f) % 6f);
            else if (max == g) h = 60f * (((b - r) / d) + 2f);
            else               h = 60f * (((r - g) / d) + 4f);
            if (h < 0) h += 360f;
        }

        // ---------- keep largest component (4-neighborhood) ----------
        private static void KeepLargestComponentInPlace(bool[,] mask)
        {
            int rows = mask.GetLength(0), cols = mask.GetLength(1);
            var visited = new bool[rows, cols];
            int bestCount = 0; List<(int r,int c)> best = new();

            int[] dr = { 1,-1, 0, 0 };
            int[] dc = { 0, 0, 1,-1 };

            for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
            {
                if (!mask[r, c] || visited[r, c]) continue;

                var comp = new List<(int,int)>();
                var q = new Queue<(int,int)>();
                q.Enqueue((r, c)); visited[r, c] = true;

                while (q.Count > 0)
                {
                    var (rr, cc) = q.Dequeue();
                    comp.Add((rr, cc));
                    for (int k = 0; k < 4; k++)
                    {
                        int nr = rr + dr[k], nc = cc + dc[k];
                        if ((uint)nr >= (uint)rows || ((uint)nc >= (uint)cols)) continue;
                        if (visited[nr, nc] || !mask[nr, nc]) continue;
                        visited[nr, nc] = true; q.Enqueue((nr, nc));
                    }
                }

                if (comp.Count > bestCount) { bestCount = comp.Count; best = comp; }
            }

            for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                mask[r, c] = false;

            foreach (var (r, c) in best) mask[r, c] = true;
        }
    }
}
