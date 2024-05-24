<div align="center">
	<img src="https://github.com/ecurtiss/CatRom/blob/master/docs/logo-light.svg#gh-light-mode-only" height="180" alt="CatRom logo"/>
	<img src="https://github.com/ecurtiss/CatRom/blob/master/docs/logo-dark.svg#gh-dark-mode-only" height="180" alt="CatRom logo"/>
	<hr/>
</div>

Creates [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).

The Catmull-Rom spline (CatRom) is a cousin of the popular Bézier curve that is guaranteed to pass through all of its control points.

<img src="docs/tube.png" height="300"/>

## How to use
The CatRom constructor takes 3 arguments:
1. `points`: An array of Vector2s, Vector3s, or CFrames.
2. `alpha` [optional]: A number (usually) in [0, 1] that determines the "parametrization" of the spline; defaults to 0.5.
3. `tension` [optional]: A number (usually) in [0, 1] that determines how loose the spline is; defaults to 0.

The default `alpha` of 0.5 is the only way to avoid cusps and loops, as shown [in this paper](http://www.cemyuksel.com/research/catmullrom_param/).

## API
***Note:*** *For each `Solve` method, there exists a `SolveUnitSpeed` counterpart that does the same thing but on a unit-speed parametrization of the spline, i.e., a parametrization of the spline where the velocity is always 1. Another way to say this is that if you call `SolveUnitSpeedPosition` at 100 equally spaced times, you will get back 100 equally spaced points (measured by arc length). Be aware that the `UnitSpeed` methods are slower to compute.*
```lua
CatRom.new(points: array, alpha: number?, tension: number?)
```
Creates a new Catmull-Rom spline from a list of Vector2s, Vector3s, or CFrames.
```lua
CatRom:SolvePosition(t: number)
```
Returns the position of the spline at time `t`.
```lua
CatRom:SolveCFrame(t: number)
```
Returns a CFrame at position `SolvePosition(t)` that faces in the direction of `SolveTangent(t)`.
```lua
CatRom:SolveRotCFrame(t: number)
```
Returns a CFrame at position `SolvePosition(t)` with orientation interpolated between the previous and next control points (provided your control points are CFrames). The interpolation uses spherical quadrangle interpolation.
```lua
CatRom:SolveVelocity(t: number)
```
Returns the velocity of the spline at time `t`.
```lua
CatRom:SolveAcceleration(t: number)
```
Returns the acceleration of the spline at time `t`.
```lua
CatRom:SolveTangent(t: number)
```
Returns the forward-facing, unit-length tangent vector at time `t`.
```lua
CatRom:SolveNormal(t: number)
```
Returns a unit-length vector at time `t` that is perpendicular to the spline and points in the direction of curvature. Returns `Vector3.new(nan, nan, nan)` when the curvature is 0.
```lua
CatRom:SolveBinormal(t: number)
```
Returns the cross product of `SolveTangent(t)` and `SolveNormal(t)`.
```lua
CatRom:SolveCurvature(t: number)
```
Returns the curvature of the spline at time `t`.
```lua
CatRom:SolveLength(a: number?, b: number?)
```
Returns the arc length between the points at times `a` and `b`.
```lua
CatRom:PrecomputeArcLengthParams(numIntervals: number?)
```
Computes a lookup table that makes `SolveUnitSpeed` calculations faster but less accurate.

## Performance Tips
### 1. `UnitSpeed` methods
If you are calling many `UnitSpeed` methods, you should call `PrecomputeArcLengthParams()` immediately after construction. This will make your `UnitSpeed` calls less accurate but cheaper to compute. The accuracy can be further tuned using the `numIntervals` argument; lower is faster and less accurate, higher is slower and more accurate (defaults to 16).

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
local spline, splineT = catRom:GetSplineFromTime(t)
spline:SolvePosition(splineT)
spline:SolveVelocity(splineT)
spline:SolveTangent(splineT)
```