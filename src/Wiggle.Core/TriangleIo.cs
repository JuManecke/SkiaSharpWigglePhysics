using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using SkiaSharp;

namespace Wiggle.Core
{
    public record TriangleDto(float Ax,float Ay,float Bx,float By,float Cx,float Cy,uint Argb);

    public static class TriangleIo
    {
        public static void Save(string path, IEnumerable<Triangle> tris)
        {
            var list = new List<TriangleDto>();
            foreach (var t in tris)
                list.Add(new TriangleDto(
                    t.A.X, t.A.Y, t.B.X, t.B.Y, t.C.X, t.C.Y,
                    ((uint)t.FillColor.Alpha << 24) | ((uint)t.FillColor.Red << 16) | ((uint)t.FillColor.Green << 8) | t.FillColor.Blue
                ));

            var json = JsonSerializer.Serialize(list, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(path, json);
        }

        public static List<Triangle> Load(string path)
        {
            var json = File.ReadAllText(path);
            var list = JsonSerializer.Deserialize<List<TriangleDto>>(json) ?? new();
            var result = new List<Triangle>(list.Count);
            foreach (var d in list)
            {
                var color = new SKColor(
                    (byte)((d.Argb >> 16) & 0xFF), // R
                    (byte)((d.Argb >> 8)  & 0xFF), // G
                    (byte)( d.Argb        & 0xFF), // B
                    (byte)((d.Argb >> 24) & 0xFF)  // A
                );
                result.Add(new Triangle(
                    new SKPoint(d.Ax, d.Ay),
                    new SKPoint(d.Bx, d.By),
                    new SKPoint(d.Cx, d.Cy),
                    color
                ));
            }
            return result;
        }

        public static List<Triangle> TriangulateFromImage(
            string imagePath, int canvasW, int canvasH,
            int margin = 20, int step = 32)
        {
            var result = new List<Triangle>();
            using var src = SKBitmap.Decode(imagePath);
            if (src == null) return result;
            
            float maxW = canvasW - 2 * margin;
            float maxH = canvasH - 2 * margin;
            float scale = MathF.Min(maxW / src.Width, maxH / src.Height);
            if (scale <= 0) scale = 1f;

            int sw = Math.Max(1, (int)MathF.Round(src.Width * scale));
            int sh = Math.Max(1, (int)MathF.Round(src.Height * scale));
            using var scaled = src.Resize(new SKImageInfo(sw, sh), SKFilterQuality.Medium) ?? src;

            float left = margin + (maxW - sw) * 0.5f;
            float top  = margin + (maxH - sh) * 0.5f;
            
            SKColor GuessBackground(SKBitmap bmp)
            {
                var buckets = new Dictionary<int, (int Count, SKColor Sample)>();
                void Add(SKColor c)
                {
                    int key = ((c.Red >> 3) << 10) | ((c.Green >> 3) << 5) | (c.Blue >> 3);
                    if (buckets.TryGetValue(key, out var v)) buckets[key] = (v.Count + 1, v.Sample);
                    else buckets[key] = (1, c);
                }
                int w = bmp.Width, h = bmp.Height;
                int stride = Math.Max(1, Math.Min(w, h) / 64);
                for (int x = 0; x < w; x += stride) { Add(bmp.GetPixel(x, 0)); Add(bmp.GetPixel(x, h - 1)); }
                for (int y = 0; y < h; y += stride) { Add(bmp.GetPixel(0, y)); Add(bmp.GetPixel(w - 1, y)); }
                int best = -1; SKColor sample = SKColors.White;
                foreach (var kv in buckets) if (kv.Value.Count > best) { best = kv.Value.Count; sample = kv.Value.Sample; }
                return sample;
            }
            bool IsBackground(SKColor c, SKColor bg)
            {
                if (c.Alpha < 25) return true;
                int dr = c.Red - bg.Red, dg = c.Green - bg.Green, db = c.Blue - bg.Blue;
                return (dr * dr + dg * dg + db * db) < 35 * 35;
            }

            var bg = GuessBackground(scaled);
            step = Math.Clamp(step <= 0 ? 32 : step, 12, Math.Max(12, Math.Min(sw, sh) / 10));

            for (int y = 0; y + step <= sh; y += step)
            {
                for (int x = 0; x + step <= sw; x += step)
                {
                    var p00 = new SKPoint(left + x,         top + y);
                    var p10 = new SKPoint(left + x + step,  top + y);
                    var p01 = new SKPoint(left + x,         top + y + step);
                    var p11 = new SKPoint(left + x + step,  top + y + step);

                    bool diag = (((x / step) + (y / step)) & 1) == 0;

                    void TryAdd(SKPoint A, SKPoint B, SKPoint C)
                    {
                        float cx = (A.X + B.X + C.X) / 3f;
                        float cy = (A.Y + B.Y + C.Y) / 3f;
                        int sx = (int)MathF.Round(cx - left);
                        int sy = (int)MathF.Round(cy - top);
                        if (sx < 0 || sy < 0 || sx >= scaled.Width || sy >= scaled.Height) return;

                        var col = scaled.GetPixel(sx, sy);
                        if (IsBackground(col, bg)) return;
                        
                        result.Add(new Triangle(A, B, C, col));
                    }

                    if (diag) { TryAdd(p00, p10, p11); TryAdd(p00, p11, p01); }
                    else      { TryAdd(p00, p10, p01); TryAdd(p10, p11, p01); }
                }
            }

            return result;
        }

    }
}