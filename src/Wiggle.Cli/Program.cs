using System;
using System.IO;
using System.Diagnostics;
using System.Collections.Generic;
using SkiaSharp;
using Wiggle.Core;
using Wiggle.Core.Presets;
using Wiggle.Core.Importers;

namespace Wiggle.Cli
{
    internal static class Program
    {
        static int Main(string[] args)
        {
            string inputPath = null;
            int i;
            
            for (i = 0; i < args.Length; i++)
            {
                string a = args[i];
                if (a == "--help" || a == "-h")
                {
                    PrintUsage();
                    return 0;
                }
                if (a.StartsWith("--in="))
                {
                    inputPath = a.Substring(5);
                }
                else if (a == "--in")
                {
                    if (i + 1 < args.Length) inputPath = args[i + 1];
                }
                else if (!a.StartsWith("--") && inputPath == null)
                {
                    inputPath = a;
                }
            }

            try
            {
                string root = FindRepoRoot();
                string imagesDir = Path.Combine(root, "Images");
                Directory.CreateDirectory(imagesDir);

                string stem = "goose_out";
                if (!string.IsNullOrWhiteSpace(inputPath))
                {
                    stem = Path.GetFileNameWithoutExtension(inputPath);
                }

                const int FrameStroke = 20;

                SKBitmap srcBmp = null;
                List<Triangle> triangles;

                if (!string.IsNullOrWhiteSpace(inputPath))
                {
                    if (!File.Exists(inputPath))
                    {
                        Console.Error.WriteLine("[ERROR] Input file not found: " + inputPath);
                        return 2;
                    }

                    srcBmp = SKBitmap.Decode(inputPath);
                    if (srcBmp == null)
                    {
                        Console.Error.WriteLine("[ERROR] Could not decode image: " + inputPath);
                        return 2;
                    }

                    triangles = ImageToTriangles.FromImage(
                        srcBmp,
                        18,   // step: smaller = more triangles
                        28,   // whiteRgbTol
                        0.20f,// whiteSatMax
                        0.94f // whiteValMin
                    );
                }
                else
                {
                    triangles = Goose.Create();
                }

                if (triangles == null || triangles.Count == 0)
                {
                    Console.Error.WriteLine("[ERROR] No foreground detected.");
                    return 2;
                }

                // Canvas size = image size + frame (for preview only)
                int innerW = srcBmp != null ? srcBmp.Width : 820;
                int innerH = srcBmp != null ? srcBmp.Height : 820;
                int canvasW = innerW + 2 * FrameStroke;
                int canvasH = innerH + 2 * FrameStroke;

                SKRect innerBounds = new SKRect(FrameStroke, FrameStroke, canvasW - FrameStroke, canvasH - FrameStroke);

                List<Triangle> fitted = FitTrianglesToRect(triangles, innerBounds);
                SoftBody body = SoftBody.FromTriangles(fitted, innerBounds);

                // settle a bit
                int steps;
                for (steps = 0; steps < 60; steps++)
                {
                    body.SimulationSteps(1f / 60f, new Vec2(0, 0));
                }

                // Render preview
                using (SKBitmap bmp = new SKBitmap(canvasW, canvasH))
                using (SKCanvas canvas = new SKCanvas(bmp))
                {
                    canvas.Clear(SKColors.DarkGray);

                    // baseline at lowest vertex
                    float baselineY = FindLowestVertexY(body);
                    using (SKPaint baseline = new SKPaint())
                    {
                        baseline.Color = SKColors.Red;
                        baseline.IsStroke = true;
                        baseline.StrokeWidth = FrameStroke;
                        baseline.IsAntialias = true;
                        canvas.DrawLine(0, baselineY, canvasW, baselineY, baseline);
                    }

                    // mesh
                    body.DrawMesh(canvas);

                    // frame around
                    using (SKPaint frame = new SKPaint())
                    {
                        frame.Color = SKColors.Red;
                        frame.IsStroke = true;
                        frame.StrokeWidth = FrameStroke;
                        frame.IsAntialias = true;
                        canvas.DrawRect(0, 0, canvasW, canvasH, frame);
                    }

                    // save PNG
                    string outPng = Path.Combine(imagesDir, stem + ".png");
                    using (SKImage img = SKImage.FromBitmap(bmp))
                    using (SKData data = img.Encode(SKEncodedImageFormat.Png, 90))
                    using (FileStream fs = File.Open(outPng, FileMode.Create, FileAccess.Write))
                    {
                        data.SaveTo(fs);
                    }
                    Console.WriteLine("Saved PNG  - " + outPng);
                }

                // save original triangles JSON (unfitted)
                string outJson = Path.Combine(imagesDir, stem + ".json");
                TriangleIo.Save(outJson, triangles);
                Console.WriteLine("Saved JSON - " + outJson);

                // try open the PNG
                TryOpen(outJson, FindRepoRoot());

                return 0;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine(ex.Message);
                return 1;
            }
        }
        
        private static float FindLowestVertexY(SoftBody body)
        {
            float y = 0f;
            bool first = true;
            int i;
            for (i = 0; i < body.Vertices.Count; i++)
            {
                float vy = body.Vertices[i].CurrentPosition.Y;
                if (first || vy > y)
                {
                    y = vy;
                    first = false;
                }
            }
            return y;
        }

        private static void TryOpen(string jsonPath, string root)
        {
            try
            {
                string proj = Path.Combine(root, "GooseWiggle.csproj");
                if (File.Exists(proj))
                {
                    ProcessStartInfo psi = new ProcessStartInfo("dotnet", "run --project \"" 
                                                                          + proj + "\" -- --tri=\"" + jsonPath + "\"");
                    psi.UseShellExecute = false;
                    psi.CreateNoWindow = true;
                    psi.WorkingDirectory = Path.GetDirectoryName(proj);
                    Process.Start(psi);
                }
            }
            catch { }
        }

        private static string FindRepoRoot()
        {
            string[] starts = new string[]
            {
                Directory.GetCurrentDirectory(),
                AppContext.BaseDirectory
            };

            int i;
            for (i = 0; i < starts.Length; i++)
            {
                DirectoryInfo dir = new DirectoryInfo(starts[i]);
                int depth = 0;
                while (dir != null && depth < 10)
                {
                    string sln = Path.Combine(dir.FullName, "SkiaSharpWigglePhysics.sln");
                    if (File.Exists(sln)) return dir.FullName;
                    dir = dir.Parent;
                    depth++;
                }
            }
            return Directory.GetCurrentDirectory();
        }

        private static List<Triangle> FitTrianglesToRect(List<Triangle> src, SKRect target)
        {
            float minX, minY, maxX, maxY;
            GetBounds(src, out minX, out minY, out maxX, out maxY);

            float w = maxX - minX;
            float h = maxY - minY;
            if (w <= 0) w = 1;
            if (h <= 0) h = 1;

            float sx = target.Width / w;
            float sy = target.Height / h;
            float s = sx < sy ? sx : sy;

            float contentW = s * w;
            float contentH = s * h;
            float ox = target.Left + (target.Width - contentW) * 0.5f - s * minX;
            float oy = target.Top + (target.Height - contentH) * 0.5f - s * minY;

            List<Triangle> dst = new List<Triangle>(src.Count);
            int i;
            for (i = 0; i < src.Count; i++)
            {
                Triangle t = src[i];
                SKPoint A = new SKPoint(t.A.X * s + ox, t.A.Y * s + oy);
                SKPoint B = new SKPoint(t.B.X * s + ox, t.B.Y * s + oy);
                SKPoint C = new SKPoint(t.C.X * s + ox, t.C.Y * s + oy);
                dst.Add(new Triangle(A, B, C, t.FillColor));
            }
            return dst;
        }

        private static void GetBounds(List<Triangle> tris,
                                      out float minX, out float minY, out float maxX, out float maxY)
        {
            minX = float.MaxValue;
            minY = float.MaxValue;
            maxX = float.MinValue;
            maxY = float.MinValue;

            int i;
            for (i = 0; i < tris.Count; i++)
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

            if (minX == float.MaxValue) { minX = 0; minY = 0; maxX = 1; maxY = 1; }
        }

        private static void PrintUsage()
        {
            Console.WriteLine("Wiggle.Cli creates an image that triangulates the body and is kept in a preview PNG");
            Console.WriteLine("Usage: dotnet run --project .\\src\\Wiggle.Cli\\Wiggle.Cli.csproj -- .\\Images\\your.png");
        }
    }
}
