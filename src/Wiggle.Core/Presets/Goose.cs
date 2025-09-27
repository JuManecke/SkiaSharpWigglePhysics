using System.Collections.Generic;
using SkiaSharp;

namespace Wiggle.Core.Presets
{
    public static class Goose
    {
        /// <summary>
        /// goose-body triangles, positions and colors (EXACT copy)
        /// </summary>
        public static List<Triangle> Create()
        {
            var sourceTriangles = new List<Triangle>();

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

            return sourceTriangles;
        }
    }
}
