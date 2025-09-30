using System;
using System.Collections.Generic;
using SkiaSharp;

namespace Wiggle.Core.Importers
{
    public static class ImageToTriangles
    {
        // Foreground-only triangulation (white background gets ignored).
        public static List<Triangle> FromImage(
            SKBitmap src,
            int step,
            int whiteRgbTol,
            float whiteSatMax,
            float whiteValMin)
        {
            if (src == null) throw new ArgumentNullException("src");
            if (step < 4) step = 4;

            int cols = (int)Math.Ceiling(src.Width / (float)step);
            int rows = (int)Math.Ceiling(src.Height / (float)step);
            if (cols < 1) cols = 1;
            if (rows < 1) rows = 1;

            bool[,] fg = new bool[rows, cols];
            
            int r, c;
            for (r = 0; r < rows; r++)
            {
                for (c = 0; c < cols; c++)
                {
                    int cx = Clamp(c * step + step / 2, 0, src.Width - 1);
                    int cy = Clamp(r * step + step / 2, 0, src.Height - 1);
                    SKColor col = src.GetPixel(cx, cy);
                    fg[r, c] = IsNonWhite(col, whiteRgbTol, whiteSatMax, whiteValMin);
                }
            }
            
            KeepLargestComponentInPlace(fg);

            List<Triangle> tris = new List<Triangle>();

            // build triangles per cell
            for (r = 0; r < rows - 1; r++)
            {
                for (c = 0; c < cols - 1; c++)
                {
                    int count =
                        (fg[r, c] ? 1 : 0) +
                        (fg[r, c + 1] ? 1 : 0) +
                        (fg[r + 1, c] ? 1 : 0) +
                        (fg[r + 1, c + 1] ? 1 : 0);
                    if (count < 2) continue;

                    float x0 = c * step;
                    float y0 = r * step;
                    float x1 = Math.Min(src.Width, (c + 1) * step);
                    float y1 = Math.Min(src.Height, (r + 1) * step);
                    
                    SKPoint A0 = new SKPoint(x0, y0);
                    SKPoint A1 = new SKPoint(x1, y0);
                    SKPoint A2 = new SKPoint(x0, y1);
                    SKColor fillA;
                    if (TriangleIsForeground(src, A0, A1, A2, whiteRgbTol, whiteSatMax, whiteValMin, out fillA))
                    {
                        tris.Add(new Triangle(A0, A1, A2, fillA));
                    }
                    
                    SKPoint B0 = new SKPoint(x1, y0);
                    SKPoint B1 = new SKPoint(x1, y1);
                    SKPoint B2 = new SKPoint(x0, y1);
                    SKColor fillB;
                    if (TriangleIsForeground(src, B0, B1, B2, whiteRgbTol, whiteSatMax, whiteValMin, out fillB))
                    {
                        tris.Add(new Triangle(B0, B1, B2, fillB));
                    }
                }
            }

            if (tris.Count == 0)
            {
                throw new InvalidOperationException("No foreground triangles — adjust you image.");
            }

            return tris;
        }
        
        private static bool TriangleIsForeground(
            SKBitmap src, SKPoint p0, SKPoint p1, SKPoint p2,
            int rgbTol, float whiteSatMax, float whiteValMin, out SKColor fill)
        {
            SKColor c0 = Sample(src, p0);
            SKColor c1 = Sample(src, p1);
            SKColor c2 = Sample(src, p2);
            SKPoint pc = new SKPoint((p0.X + p1.X + p2.X) / 3f, (p0.Y + p1.Y + p2.Y) / 3f);
            SKColor cc = Sample(src, pc);

            bool s0 = IsNonWhite(c0, rgbTol, whiteSatMax, whiteValMin);
            bool s1 = IsNonWhite(c1, rgbTol, whiteSatMax, whiteValMin);
            bool s2 = IsNonWhite(c2, rgbTol, whiteSatMax, whiteValMin);
            bool sc = IsNonWhite(cc, rgbTol, whiteSatMax, whiteValMin);

            int votes = (s0 ? 1 : 0) + (s1 ? 1 : 0) + (s2 ? 1 : 0) + (sc ? 1 : 0);
            bool keep = votes >= 3;

            fill = AverageNonWhite(c0, c1, c2, cc, rgbTol, whiteSatMax, whiteValMin, cc);
            return keep;
        }

        private static SKColor Sample(SKBitmap bmp, SKPoint p)
        {
            int x = Clamp((int)Math.Round(p.X), 0, bmp.Width - 1);
            int y = Clamp((int)Math.Round(p.Y), 0, bmp.Height - 1);
            return bmp.GetPixel(x, y);
        }

        private static SKColor AverageNonWhite(SKColor a, SKColor b, SKColor c, SKColor d,
                                               int rgbTol, float whiteSatMax, float whiteValMin,
                                               SKColor fallback)
        {
            long r = 0, g = 0, bl = 0;
            int n = 0;

            if (IsNonWhite(a, rgbTol, whiteSatMax, whiteValMin)) { r += a.Red; g += a.Green; bl += a.Blue; n++; }
            if (IsNonWhite(b, rgbTol, whiteSatMax, whiteValMin)) { r += b.Red; g += b.Green; bl += b.Blue; n++; }
            if (IsNonWhite(c, rgbTol, whiteSatMax, whiteValMin)) { r += c.Red; g += c.Green; bl += c.Blue; n++; }
            if (IsNonWhite(d, rgbTol, whiteSatMax, whiteValMin)) { r += d.Red; g += d.Green; bl += d.Blue; n++; }

            if (n == 0) return fallback;
            return new SKColor((byte)(r / n), (byte)(g / n), (byte)(bl / n), 255);
        }
        
        private static bool IsNonWhite(SKColor c, int rgbTol, float whiteSatMax, float whiteValMin)
        {
            if (c.Alpha < 16) return false;

            int dr = 255 - c.Red;
            int dg = 255 - c.Green;
            int db = 255 - c.Blue;
            int d2 = dr * dr + dg * dg + db * db;
            bool nearWhiteRgb = d2 <= rgbTol * rgbTol;

            float h, s, v;
            RgbToHsv(c, out h, out s, out v);
            bool whiteishHsv = (s <= whiteSatMax && v >= whiteValMin);

            return !(nearWhiteRgb || whiteishHsv);
        }

        private static void RgbToHsv(SKColor c, out float h, out float s, out float v)
        {
            float r = c.Red / 255f;
            float g = c.Green / 255f;
            float b = c.Blue / 255f;

            float max = Math.Max(r, Math.Max(g, b));
            float min = Math.Min(r, Math.Min(g, b));
            v = max;

            float d = max - min;
            s = max == 0 ? 0 : d / max;

            if (d == 0) { h = 0; return; }

            if (max == r)
            {
                h = 60f * (((g - b) / d + 6f) % 6f);
            }
            else if (max == g)
            {
                h = 60f * (((b - r) / d) + 2f);
            }
            else
            {
                h = 60f * (((r - g) / d) + 4f);
            }
            if (h < 0) h += 360f;
        }
        
        private static void KeepLargestComponentInPlace(bool[,] mask)
        {
            int rows = mask.GetLength(0);
            int cols = mask.GetLength(1);

            bool[,] visited = new bool[rows, cols];
            int bestCount = 0;
            System.Collections.Generic.List<System.Tuple<int, int>> best = new System.Collections.Generic.List<System.Tuple<int, int>>();

            int[] dr = new int[] { 1, -1, 0, 0 };
            int[] dc = new int[] { 0, 0, 1, -1 };

            int r, c;
            for (r = 0; r < rows; r++)
            {
                for (c = 0; c < cols; c++)
                {
                    if (!mask[r, c] || visited[r, c]) continue;

                    System.Collections.Generic.List<System.Tuple<int, int>> comp =
                        new System.Collections.Generic.List<System.Tuple<int, int>>();
                    System.Collections.Generic.Queue<System.Tuple<int, int>> q =
                        new System.Collections.Generic.Queue<System.Tuple<int, int>>();

                    visited[r, c] = true;
                    q.Enqueue(new System.Tuple<int, int>(r, c));

                    while (q.Count > 0)
                    {
                        System.Tuple<int, int> node = q.Dequeue();
                        int rr = node.Item1;
                        int cc = node.Item2;
                        comp.Add(node);

                        int k;
                        for (k = 0; k < 4; k++)
                        {
                            int nr = rr + dr[k];
                            int nc = cc + dc[k];
                            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                            if (visited[nr, nc] || !mask[nr, nc]) continue;
                            visited[nr, nc] = true;
                            q.Enqueue(new System.Tuple<int, int>(nr, nc));
                        }
                    }

                    if (comp.Count > bestCount)
                    {
                        bestCount = comp.Count;
                        best = comp;
                    }
                }
            }
            
            for (r = 0; r < rows; r++)
            {
                for (c = 0; c < cols; c++)
                {
                    mask[r, c] = false;
                }
            }
            
            int i;
            for (i = 0; i < best.Count; i++)
            {
                var t = best[i];
                mask[t.Item1, t.Item2] = true;
            }
        }

        private static int Clamp(int v, int min, int max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }
    }
}
