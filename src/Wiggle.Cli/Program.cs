using System;
using System.IO;
using SkiaSharp;
using Wiggle.Core;
using Wiggle.Core.Presets;


class Program
{
    static int Main(string[] args)
    {
        try
        {
            var outDir = Path.Combine(AppContext.BaseDirectory, "artifacts");
            Directory.CreateDirectory(outDir);
            var outPng = Path.Combine(outDir, "goose_out.png");


            const int W = 820, H = 820;
            var bounds = new SKRect(20, 20, W - 20, H - 20);


            var triangles = Goose.Create();
            var body = SoftBody.FromTriangles(triangles, bounds);


            for (int i = 0; i < 60; i++)
                body.SimulationSteps(1f / 60f, new Vec2(0, 0));


            using var bmp = new SKBitmap(W, H);
            using var canvas = new SKCanvas(bmp);
            canvas.Clear(SKColors.White);
            using (var frame = new SKPaint { Color = SKColors.Red, IsStroke = true, StrokeWidth = 20, IsAntialias = true })
                canvas.DrawRect(bounds, frame);
            body.DrawMesh(canvas);


            using var img = SKImage.FromBitmap(bmp);
            using var data = img.Encode(SKEncodedImageFormat.Png, 90);
            using var fs = File.Open(outPng, FileMode.Create, FileAccess.Write);
            data.SaveTo(fs);


            Console.WriteLine($"Saved PNG → {outPng}");
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.ToString());
            return 1;
        }
    }
}