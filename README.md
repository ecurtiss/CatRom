<div align="center">
	<img src="https://github.com/EthanCurtiss/CatRom/blob/master/docs/logo-light.svg#gh-light-mode-only" height="180" alt="CatRom logo"/>
	<img src="https://github.com/EthanCurtiss/CatRom/blob/master/docs/logo-dark.svg#gh-dark-mode-only" height="180" alt="CatRom logo"/>
	<hr/>
</div>

Creates [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).

The Catmull-Rom spline (CatRom) is a cousin of the popular Bézier curve, with the key difference that CatRoms are guaranteed to pass through their control points. This allows them to chain together predictably and intuitively.

<img src="docs/tube.png" height="300"/>

## How to use
The CatRom constructor takes 3 arguments:
1. `points`: An array of Vector2s, Vector3s, or CFrames.
2. `alpha` [optional]: A number (usually) in [0, 1] that determines the "parameterization" of the spline; defaults to 0.5.
3. `tension` [optional]: A number (usually) in [0, 1] that determines how loose the spline is; defaults to 0.

The default `alpha` of 0.5 is the only way to avoid cusps and loops, as shown [in this paper](http://www.cemyuksel.com/research/catmullrom_param/).

## API
***Note:*** *For each `Solve` method, there exists a `SolveUniform` counterpart that spaces the input(s) uniformly along the curve. Be aware that the uniform methods are slower to compute.*
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
