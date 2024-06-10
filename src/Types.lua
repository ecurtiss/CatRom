export type Point = Vector2 | Vector3 | CFrame
export type PointType = "Vector2" | "Vector3" | "CFrame"
export type Vector = Vector2 | Vector3
export type Quaternion = { number } -- {w, x, y, z}

export type CatRom = typeof(setmetatable(
	{} :: {
		knots: {number},
		length: number,
		points: {Point},
		splines: {Spline}
	},
	{} :: CatRomMt
))

export type CatRomMt = {
	__index: CatRomMt,

	new: (points: {Point}, alpha: number?, tension: number?, loops: boolean?) -> CatRom,
	
	-- Piecewise methods
	GetSplineAtTime: (self: CatRom, t: number) -> (Spline, number, number),
	PrecomputeUnitSpeedData: (
		self: CatRom,
		when: "now" | "on demand",
		strategy: "fast" | "accurate",
		degree: number?
	) -> (),
	SolveLength: (self: CatRom, from: number?, to: number?) -> number,
	SolveBulk: (
		self: CatRom,
		f: (spline: Spline, t: number) -> (),
		numSamples: number,
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> (),
	SolveBoundingBox: (self: CatRom) -> (Vector, Vector),
	CreateTween: (
		self: CatRom,
		tweenInfo: TweenInfo,
		callback: (spline: Spline, time: number) -> (),
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> Tween,

	-- Rotation-minimzing frame methods
	PrecomputeRMFs: (
		self: CatRom,
		numFramesPerSpline: number?,
		firstSplineIndex: number?,
		lastSplineIndex: number?
	) -> (),
	SolveCFrameRMF: (
		self: CatRom,
		t: number,
		unitSpeed: boolean?,
		prevFrame: CFrame?,
		numFramesPerSpline: number?
	) -> CFrame,
	GetParallelTransportInterpolant: (
		self: CatRom,
		data: Vector3 | CFrame,
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> (t: number) -> Vector3 | CFrame,
	GetNormalVectorInterpolant: (
		self: CatRom,
		from: number,
		fromVector: Vector3,
		to: number,
		toVector: Vector3,
		unitSpeed: boolean?
	) -> (t: number) -> Vector3,

	-- Proxy methods
	SolvePosition:         (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveVelocity:         (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveAcceleration:     (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveJerk:             (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveTangent:          (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveNormal:           (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveBinormal:         (self: CatRom, t: number, unitSpeed: boolean?) -> Vector3,
	SolveCurvature:        (self: CatRom, t: number, unitSpeed: boolean?) -> number,
	SolveTorsion:          (self: CatRom, t: number, unitSpeed: boolean?) -> number,
	SolveCFrameLookAlong:  (self: CatRom, t: number, unitSpeed: boolean?, upVector: Vector3?) -> CFrame,
	SolveCFrameFrenet:     (self: CatRom, t: number, unitSpeed: boolean?) -> CFrame,
	SolveCFrameSquad:      (self: CatRom, t: number, unitSpeed: boolean?) -> CFrame,
}

export type Spline = typeof(setmetatable(
	{} :: {
		cheb: Chebyshev?,
		chebDegree: number?,
		chebIsLUT: boolean?,
		length: number,
		rmfLUT: {CFrame}?,
		type: PointType,

		-- Coefficient vectors for position/velocity/acceleration/jerk polynomials
		a: Vector,
		b: Vector,
		c: Vector,
		d: Vector,

		-- Rotations (nil if type is Vector2 or Vector3)
		q0: Quaternion?,
		q1: Quaternion?,
		q2: Quaternion?,
		q3: Quaternion?,
	},
	{} :: SplineMt
))

export type SplineMt = {
	__index: SplineMt,

	new: (
		a: Vector,
		b: Vector,
		c: Vector,
		d: Vector,
		pointType: PointType,
		q0: Quaternion?,
		q1: Quaternion?,
		q2: Quaternion,
		q3: Quaternion,
		length: number?
	) -> Spline,

	-- Basic methods
	SolvePosition:     (self: Spline, t: number) -> Vector,
	SolveVelocity:     (self: Spline, t: number) -> Vector,
	SolveAcceleration: (self: Spline, t: number) -> Vector,
	SolveJerk:         (self: Spline) -> Vector,
	SolveTangent:      (self: Spline, t: number) -> Vector,
	SolveNormal:       (self: Spline, t: number) -> Vector,
	SolveBinormal:     (self: Spline, t: number) -> Vector3,
	SolveCurvature:    (self: Spline, t: number) -> number,
	SolveTorsion:      (self: Spline, t: number) -> number,

	-- Moving frame methods
	SolveCFrameLookAlong: (self: Spline, t: number, upVector: Vector3?) -> CFrame,
	SolveCFrameFrenet:    (self: Spline, t: number) -> CFrame,
	SolveCFrameSquad:     (self: Spline, t: number) -> CFrame,
	SolveCFrameRMF:       (self: Spline, t: number, prevFrame: CFrame?) -> CFrame,
	PrecomputeRMFs: (
		self: Spline,
		numFramesPerSpline: number,
		initialFrame: CFrame
	) -> (),
	
	-- Numerical methods
	SolveLength: (self: Spline, from: number?, to: number?) -> number,
	SolveBoundingBox: (self: Spline) -> (Vector, Vector),
	
	-- Arc length reparametrization methods
	Reparametrize: (self: Spline, s: number) -> number,
	PrecomputeUnitSpeedData: (self: Spline, precomputeNow: boolean, useChebAsLUT: boolean, degree: number) -> (),
	_ReparametrizeNewtonBisection: (self: Spline, s: number) -> number,
	_GetChebyshevInterpolant: (self: Spline, degree: number) -> Chebyshev,
}

export type Chebyshev = typeof(setmetatable(
	{} :: {
		grid: {number},
		gridValues: {number},
	},
	{} :: ChebyshevMt
))

export type ChebyshevMt = {
	__index: ChebyshevMt,

	new: (f: (number) -> number, degree: number) -> Chebyshev,

	Evaluate: (self: Chebyshev, x: number) -> number,
	GetSolveBounds: (self: Chebyshev, y: number) -> (number, number, number, number),
	Solve: (self: Chebyshev, y: number) -> number,
	Invert: (self: Chebyshev) -> Chebyshev,
}

return nil