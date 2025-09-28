# SkiaSharpWigglePhysics

Interactive 2D wiggle physics for triangulated images using SkiaSharp.

**What this does**
- Triangulates **only the foreground** (any pixel that is **not near-white**). White is treated as background and ignored.
- Preview PNG draws a **red frame around** the content.
- The **wiggle window** loads triangles and **fits** them into the inner play area so they always fill the window.
- Bottom red **baseline** is drawn at the lowest body line.
- Mouse: drag the whole window or parts of the body to let it wiggle.

## Projects
GooseWiggle/ # Windows app (WinForms + SkiaSharp)
src/Wiggle.Cli/ # CLI for image - triangles - PNG preview (+ auto-launch window) src/Wiggle.Core/ # Engine, importers, presets (e.g., Goose)

## Requirements

- .NET 6 SDK (or newer)
- Windows for `GooseWiggle` (WinForms)

## Quick start (Windows CMD)

From the repo root:

use the cmd

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

## How to use your own image:

1 - create a .png file, using f.e. paint. Keep the background white and dont draw a border around it. You can use all sorts of other colors though.
2 - put your .png in the folder Images
3 - run your image by using 
    dotnet run --project ".\src\Wiggle.Cli\Wiggle.Cli.csproj" -- ".\Images\my_art.png"
4 - the image with the triangles and the border should be created automaticall and a WiggleWindow should open up for you to play around with your creation

## How to trigger the default

CLI without a valid image path falls back to the built-in goose preset:
dotnet run --project ".\src\Wiggle.Cli\Wiggle.Cli.csproj"

## What to expect:

A Preview PNG: gray background, a red border around the canvas, triangles colored from the image. 
Wiggle window: same mesh, fitted to the inner area (inside the red frame). Drag it around to interact.
