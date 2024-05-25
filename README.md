<div align="center">
	<img src="https://github.com/ecurtiss/CatRom/blob/master/docs/logo-light.svg#gh-light-mode-only" height="180" alt="CatRom logo"/>
	<img src="https://github.com/ecurtiss/CatRom/blob/master/docs/logo-dark.svg#gh-dark-mode-only" height="180" alt="CatRom logo"/>
	<hr/>
</div>

Creates [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).

The Catmull-Rom spline (CatRom) is a cousin of the popular Bézier curve that passes through all of its control points.

<img src="docs/tube.png" height="300"/>

## How to use
The CatRom constructor takes 3 arguments:
1. `points`: An array of Vector2s, Vector3s, or CFrames.
2. `alpha` [optional]: A number (usually) in [0, 1] that determines the "parametrization" of the spline; defaults to 0.5.
3. `tension` [optional]: A number (usually) in [0, 1] that determines how loose the spline is; defaults to 0.

The default `alpha` of 0.5 is the only way to avoid cusps and loops, as shown [in this paper](http://www.cemyuksel.com/research/catmullrom_param/).

## API
***Note:*** *The `unitSpeed` argument in each `Solve` method determines whether the calculation uses a unit-speed parametrization of the spline, i.e., a parametrization with a constant speed of 1. Another way to say this is that if you call `SolvePosition` at 100 equally spaced times with `unitSpeed` true, you will get back 100 equally spaced points along the curve (measured by arc length). Be aware that using `unitSpeed` adds computation time.*
```lua
CatRom.new(points: array, alpha: number?, tension: number?)
```
Creates a new Catmull-Rom spline from a list of Vector2s, Vector3s, or CFrames.
```lua
CatRom:SolvePosition(t: number, unitSpeed: boolean?)
```
Returns the position of the spline at time `t`.
```lua
CatRom:SolveCFrame(t: number, unitSpeed: boolean?)
```
Returns a CFrame at position `SolvePosition(t)` that faces in the direction of `SolveTangent(t)`.
```lua
CatRom:SolveRotCFrame(t: number, unitSpeed: boolean?)
```
Returns a CFrame at position `SolvePosition(t)` with orientation interpolated between the previous and next control points (provided your control points are CFrames). The interpolation uses spherical quadrangle interpolation.
```lua
CatRom:SolveVelocity(t: number, unitSpeed: boolean?)
```
Returns the velocity of the spline at time `t`.
```lua
CatRom:SolveAcceleration(t: number, unitSpeed: boolean?)
```
Returns the acceleration of the spline at time `t`.
```lua
CatRom:SolveTangent(t: number, unitSpeed: boolean?)
```
Returns the forward-facing, unit-length tangent vector at time `t`.
```lua
CatRom:SolveNormal(t: number, unitSpeed: boolean?)
```
Returns a unit-length vector at time `t` that is perpendicular to the spline and points in the direction of curvature. Returns `Vector3.new(nan, nan, nan)` when the curvature is 0.
```lua
CatRom:SolveBinormal(t: number, unitSpeed: boolean?)
```
Returns the cross product of `SolveTangent(t)` and `SolveNormal(t)`.
```lua
CatRom:SolveCurvature(t: number, unitSpeed: boolean?)
```
Returns the curvature of the spline at time `t`.
```lua
CatRom:SolveLength(a: number?, b: number?)
```
Returns the arc length between the points at times `a` and `b`.
```lua
CatRom:PrecomputeArcLengthParams(numIntervals: number?)
```
Computes a lookup table that makes `unitSpeed` calculations faster but less accurate.

## Performance Tips
### 1. Solving with `unitSpeed`
If you are calling many Solve methods with `unitSpeed` true, you should call `PrecomputeArcLengthParams()` immediately after construction. This will make your `unitSpeed` calls less accurate but cheaper to compute. The accuracy can be further tuned using the `numIntervals` argument; lower is faster and less accurate, higher is slower and more accurate (defaults to 16).

### 2. Repeated inputs
If you are calling many methods on the *same* input like so
```lua
local t -- number in [0, 1]
local catRom -- a CatRom object
catRom:SolvePosition(t)
catRom:SolveVelocity(t)
catRom:SolveTangent(t)
```
then it is faster to instead do
```lua
local t -- number in [0, 1]
local catRom -- a CatRom object
local spline, splineTime = catRom:GetSplineAtTime(t)
spline:SolvePosition(splineTime)
spline:SolveVelocity(splineTime)
spline:SolveTangent(splineTime)
```