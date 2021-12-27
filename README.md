<div align="center">
	<img src="https://github.com/EthanCurtiss/CatRom/blob/rewrite/docs/logo-light.svg#gh-light-mode-only" height="180" alt="CatRom logo"/>
	<img src="https://github.com/EthanCurtiss/CatRom/blob/rewrite/docs/logo-dark.svg#gh-dark-mode-only" height="180" alt="CatRom logo"/>
	<hr/>
</div>
Creates [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).

The Catmull-Rom spline (CatRom) is a cousin of the popular Bézier curve, with the key difference that CatRoms are guaranteed to pass through their control points. This allows them to chain together predictably and intuitively.

<img src="docs/tube.png" height="300"/>

## How to use
The CatRom constructor takes in 3 arguments:
1. `points`: An array of all Vector2s, Vector3s, or CFrames.
2. `alpha` [optional]: A number -- usually in [0, 1] -- that determines the "parameterization" of the spline; defaults to 0.5.
3. `tension` [optional]: A number -- usually in [0, 1] -- that determines how loose the spline is; defaults to 0.

## API
```lua
CatRom.new(points: array, alpha: number?, tension: number?)
```
```lua
CatRom:SolvePosition(t: number)
```
```lua
CatRom:SolveCFrame(t: number)
```
```lua
CatRom:SolveRotCFrame(t: number)
```
```lua
CatRom:SolveVelocity(t: number)
```
```lua
CatRom:SolveAcceleration(t: number)
```
```lua
CatRom:SolveTangent(t: number)
```
```lua
CatRom:SolveNormal(t: number)
```
```lua
CatRom:SolveBinormal(t: number)
```
```lua
CatRom:SolveCurvature(t: number)
```
```lua
CatRom:SolveLength(a: number?, b: number?)
```
