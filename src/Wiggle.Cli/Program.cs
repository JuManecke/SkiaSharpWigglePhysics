using System;
using System.IO;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using SkiaSharp;
using Wiggle.Core;
using Wiggle.Core.Presets;
using Wiggle.Core.Importers;

class Program
{
    static int Main(string[] args)
    {
        string? inputPath = null;
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--in" && i + 1 < args.Length) inputPath = args[++i];
            else if (args[i].StartsWith("--in=")) inputPath = args[i].Substring(5);
            else if (!args[i].StartsWith("--") && inputPath == null) inputPath = args[i]; // positional
        }

        try
        {
            // repo root (for /Images) by finding the .sln; fallback = CWD
            string? root = FindRepoRoot();
            string imagesDir = (root != null)
                ? Path.Combine(root, "Images")
                : Path.Combine(Directory.GetCurrentDirectory(), "Images");
            Directory.CreateDirectory(imagesDir);

            string stem = !string.IsNullOrWhiteSpace(inputPath)
                ? Path.GetFileNameWithoutExtension(inputPath)
                : "goose_out";

            const int FrameStroke = 20;

            SKBitmap? srcBmp = null;
            List<Triangle> triangles;

            if (!string.IsNullOrWhiteSpace(inputPath) && File.Exists(inputPath))
            {
                srcBmp = SKBitmap.Decode(inputPath);

                // Foreground-only triangulation (ALL non-white kept)
                triangles = ImageToTriangles.FromImage(
                    srcBmp,
                    step: 18,           // denser mesh -> use smaller value (e.g., 12 or 8)
                    whiteRgbTol: 24,    // raise to remove more off-white halo (e.g., 32..40)
                    whiteSatMax: 0.20f, // raise slightly if background is mildly colored (0.25)
                    whiteValMin: 0.92f  // raise to treat very light grays as white (e.g., 0.95)
                );
            }
            else
            {
                triangles = Goose.Create();
            }

            // Canvas size = inner content + frame around it
            int innerW = srcBmp?.Width  ?? 820;
            int innerH = srcBmp?.Height ?? 820;
            int canvasW = innerW + 2 * FrameStroke;
            int canvasH = innerH + 2 * FrameStroke;

            var innerBounds = new SKRect(FrameStroke, FrameStroke, canvasW - FrameStroke, canvasH - FrameStroke);

            // Fit triangles to inner bounds for the preview image
            var fitted = FitTrianglesToRect(triangles, innerBounds);
            var body = SoftBody.FromTriangles(fitted, innerBounds);

            // settle pose slightly
            for (int i = 0; i < 60; i++)
                body.SimulationSteps(1f / 60f, new Vec2(0, 0));

            using var bmp = new SKBitmap(canvasW, canvasH);
            using var canvas = new SKCanvas(bmp);

            // background
            canvas.Clear(SKColors.DarkGray);

            // bottom red baseline (no fill)
            float baselineY = body.Vertices.Max(v => v.CurrentPosition.Y);
            using (var baseline = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStroke, IsAntialias = true })
                canvas.DrawLine(0, baselineY, canvasW, baselineY, baseline);

            // triangles
            body.DrawMesh(canvas);

            // red frame AROUND the content (does not cover it)
            using (var frame = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = FrameStroke, IsAntialias = true })
                canvas.DrawRect(0, 0, canvasW, canvasH, frame);

            // save
            var outPng  = Path.Combine(imagesDir, $"{stem}.png");
            var outJson = Path.Combine(imagesDir, $"{stem}.json");

            using var img = SKImage.FromBitmap(bmp);
            using var data = img.Encode(SKEncodedImageFormat.Png, 90);
            using (var fs = File.Open(outPng, FileMode.Create, FileAccess.Write)) data.SaveTo(fs);

            // IMPORTANT: save ORIGINAL (unfitted) triangles to JSON;
            // the window will load and fit them to its inner rect.
            TriangleIo.Save(outJson, triangles);

            Console.WriteLine($"Saved PNG  → {outPng}");
            Console.WriteLine($"Saved JSON → {outJson}");

            // open PNG
            try { Process.Start(new ProcessStartInfo(outPng) { UseShellExecute = true }); } catch { }

            // open the wiggle window on the same JSON
            TryLaunchWindow(root, outJson);
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
            return 1;
        }
    }

    // ---------- helpers ----------

    static string? FindRepoRoot()
    {
        foreach (var start in new[] { Directory.GetCurrentDirectory(), AppContext.BaseDirectory })
        {
            var dir = new DirectoryInfo(start);
            for (int depth = 0; depth < 10 && dir != null; depth++, dir = dir.Parent!)
                if (File.Exists(Path.Combine(dir.FullName, "SkiaSharpWigglePhysics.sln")))
                    return dir.FullName;
        }
        return null;
    }

    static void TryLaunchWindow(string? root, string jsonPath)
    {
        try
        {
            string proj = (root != null) ? Path.Combine(root, "GooseWiggle.csproj") : "./GooseWiggle.csproj";
            if (File.Exists(proj))
            {
                var psi = new ProcessStartInfo("dotnet", $"run --project \"{proj}\" -- --tri=\"{jsonPath}\"")
                {
                    UseShellExecute = false, CreateNoWindow = true,
                    WorkingDirectory = Path.GetDirectoryName(proj)!
                };
                Process.Start(psi);
            }
            else
            {
                Console.WriteLine($"> To open the wiggle window: dotnet run --project \"{proj}\" -- --tri=\"{jsonPath}\"");
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"> Could not auto-launch GooseWiggle. Run manually:");
            Console.WriteLine($"  dotnet run --project ./GooseWiggle.csproj -- --tri=\"{jsonPath}\"");
            Console.WriteLine(ex.Message);
        }
    }

    static List<Triangle> FitTrianglesToRect(List<Triangle> src, SKRect target)
    {
        GetBounds(src, out float minX, out float minY, out float maxX, out float maxY);
        float w = MathF.Max(1, maxX - minX);
        float h = MathF.Max(1, maxY - minY);

        float s = MathF.Min(target.Width / w, target.Height / h);
        float cw = s * w, ch = s * h;
        float ox = target.Left + (target.Width - cw) * 0.5f - s * minX;
        float oy = target.Top  + (target.Height - ch) * 0.5f - s * minY;

        static SKPoint X(SKPoint p, float s, float ox, float oy) => new SKPoint(p.X * s + ox, p.Y * s + oy);

        var dst = new List<Triangle>(src.Count);
        foreach (var t in src)
            dst.Add(new Triangle(X(t.A, s, ox, oy), X(t.B, s, ox, oy), X(t.C, s, ox, oy), t.FillColor));
        return dst;
    }

    static void GetBounds(List<Triangle> tris, out float minX, out float minY, out float maxX, out float maxY)
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
}
