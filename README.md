# CatRom
Creates [Catmull-Rom splines](https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline).

The Catmull-Rom spline (CatRom) is a cousin of the popular Bézier curve, with the key difference that CatRoms are guaranteed to pass through their control points. This allows them to chain together predictably and intuitively.

![Tube](docs/tube.png)

## How to use
This module has 2 objects: **Spline** and **Chain**.

A Spline is a CatRom defined by 4 points. A Chain is a chain of Splines. You will likely only need Chains.

The Chain constructor takes in 3 arguments:
1. `points`: An array of Vector2s, Vector3s, or CFrames.
2. `alpha` [optional]: A number -- usually in [0, 1] -- that determines the "parameterization" of the spline. You won't need this.
3. `tension` [optional]: A number -- usually in [0, 1] -- that determines how loose the spline is. Like adding slack to a rope.

## Methods
Each method listed has a `SolveUniform` counterpart. The uniform counterparts space points evently along the spline but incur a performance loss.
```lua
Chain.new(points: array, alpha: number?, tension: number?)
```
```lua
Chain:SolvePosition(t: number)
```
```lua
Chain:SolveCFrame(t: number)
```
```lua
Chain:SolveRotCFrame(t: number)
```
```lua
Chain:SolveVelocity(t: number)
```
```lua
Chain:SolveAcceleration(t: number)
```
```lua
Chain:SolveTangent(t: number)
```
```lua
Chain:SolveNormal(t: number)
```
```lua
Chain:SolveBinormal(t: number)
```
```lua
Chain:SolveCurvature(t: number)
```
```lua
Chain:SolveLength(a: number?, b: number?)
```
