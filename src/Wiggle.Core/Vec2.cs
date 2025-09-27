using System;

namespace Wiggle.Core
{
    public struct Vec2
    {
        public float X, Y;

        public Vec2(float x, float y) { X = x; Y = y; }

        public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
        public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
        public static Vec2 operator *(Vec2 a, float s) => new(a.X * s, a.Y * s);
        public static Vec2 operator *(float s, Vec2 a) => new(a.X * s, a.Y * s);
        public static Vec2 operator /(Vec2 a, float s) => new(a.X / s, a.Y / s);

        public float Length   => MathF.Sqrt(X * X + Y * Y);
        public float LengthSq => X * X + Y * Y;

        public Vec2 Normalized()
        {
            float len = Length;
            return (len > 1e-12f) ? new Vec2(X / len, Y / len) : new Vec2(0, 0);
        }

        public static float Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;
        public static Vec2 PerpCCW(Vec2 v) => new Vec2(-v.Y, v.X);
    }
}