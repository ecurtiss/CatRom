<p align="center">
	<img src="https://github.com/ecurtiss/CatRom/blob/master/img/logo-light.svg#gh-light-mode-only" height="180" alt="CatRom logo"/>
	<img src="https://github.com/ecurtiss/CatRom/blob/master/img/logo-dark.svg#gh-dark-mode-only" height="180" alt="CatRom logo"/>
	<br>
	Creates Catmull-Rom splines for Roblox
</p>

# Introduction
A [Catmull-Rom spline](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline) is a C¹ cubic spline that passes through each of its control points. It is a good choice for your project if you want a smooth curve that
1. passes through every control point and
2. does not need any extra data to sculpt it (cf. Bézier curves).

In addition to the control points, CatRom provides two parameters to adjust the spline:
1. `alpha`: A number (usually in [0, 1]) that loosely affects the curvature of the spline at the control points (default: 0.5)
2. `tension`: A number (usually in [0, 1]) that makes the spline more or less taut (default: 0)

### Reparametrization
By construction, a Catmull-Rom spline clusters points in regions of higher curvature. This effect is often visually unappealing, so
CatRom offers a second *parametrization*—called a *unit-speed* (or *arc length*) *parametrization*—that yields equally-spaced
points given equally-spaced times.

By default, passing `true` into a method with a `unitSpeed` argument performs a slow but highly accurate reparametrization. If you are calling
a high volume of methods with `unitSpeed` true, then you should instead call `CatRom:PrecomputeUnitSpeedData` beforehand; after
the initial cost of the precompute step, this method will make your reparametrizations significantly faster at the cost of accuracy.

# Installation
CatRom is available on [Wally](https://wally.run/)
```toml
catrom = "ecurtiss/catrom@0.6.0"
```
or as a `.rbxm` from the [Releases](https://github.com/ecurtiss/CatRom/releases) page.

# Getting started
Here is an annotated example to get you started. Thorough documentation can be found at [website](website).
```lua
local CatRom = require(path.to.CatRom)

local points = {...} -- A list of Vector2s, Vector3s, or CFrames
local alpha = 0.5 -- Optional; loosely affects the curvature at the control points (default: 0.5)
local tension = 0 -- Optional; makes the spline more or less taut (default: 0)

local spline = CatRom.new(points, alpha, tension)

-- Get the position at a single time
local pos = spline:SolvePosition(0.5)

-- Get the position at 100 times between 0 and 1 (inclusive)
spline:SolveBulk(function(segment, t)
	local pos = segment:SolvePosition(t)
end, 100, 0, 1)

-- Repeat the above with a unit-speed parametrization
spline:PrecomputeUnitSpeedData() -- Optional; makes the math faster for bulk computations
local pos = spline:SolvePosition(0.5, true) -- Notice the `true`
spline:SolveBulk(function(segment, t)
	local pos = segment:SolvePosition(t)
end, 100, 0, 1, true) -- Notice the `true`

-- Smoothly sweep a CFrame from time 0 to 1 with minimal twisting
local initialCF = CFrame.identity
local interpolant = spline:GetParallelTransportInterpolant(initialCF, 0, 1)
for i = 0, 100 do
	local sweptCF = interpolant(i / 100)
end
```