# SkiaSharpWigglePhysics

Interactive 2D wiggle physics for triangulated images using SkiaSharp.

**What this fork does**
- Triangulates **only the foreground** (any pixel that is **not near-white**). White is treated as background and ignored.
- Preview PNG draws a **red frame around** the content (frame does not cover triangles).
- The **wiggle window** loads triangles and **fits** them into the inner play area so they always fill the window nicely.
- Bottom red **baseline** is drawn at the lowest body vertex (same rule in preview + window).
- Mouse: click impulse + drag the body using the engines 5-param `StartDrag(...)` API.

## Projects
GooseWiggle/ # Windows app (WinForms + SkiaSharp)
src/Wiggle.Cli/ # CLI for image - triangles - PNG preview (+ auto-launch window) src/Wiggle.Core/ # Engine, importers, presets (e.g., Goose)

## Requirements

- .NET 6 SDK (or newer)
- Windows for `GooseWiggle` (WinForms). The CLI runs cross-platform.

> You may see warnings about SkiaSharp/OpenTK versions in `dotnet restore`. They are warnings, not build stops.

## Quick start (Windows CMD)

From the repo root:

```cmd
dotnet restore
dotnet build -c Release

## Make triangles from an image (non-white only) and preview PNG

dotnet run --project ".\src\Wiggle.Cli\Wiggle.Cli.csproj" -- ".\Images\devil.png"

or swap out devil.png for bird.png, pumpkin.png or your own .png 

## This Creates: 

Images/devil.json : the triangle mesh (coordinates in image space)

Images/devil.png : a preview with gray bg, red baseline, and the red frame around the content

It also tries to open the wiggle window on devil.json

## Manually open in the wiggle window

dotnet run --project ".\GooseWiggle.csproj" -- --open-latest
dotnet run --project ".\GooseWiggle.csproj" -- --tri=".\\Images\\devil.json"

## Force a specific window size:

dotnet run --project ".\GooseWiggle.csproj" -- --tri=".\\Images\\devil.json" --size=960x960
