export type Point = Vector2 | Vector3 | CFrame
export type PointType = "Vector2" | "Vector3" | "CFrame"
export type Vector = Vector2 | Vector3
export type Quaternion = { number } -- {w, x, y, z}

export type CatRom = typeof(setmetatable(
	{} :: {
		knots: {number},
		length: number,
		points: {Point},
		segments: {Segment}
	},
	{} :: CatRomMt
))

export type CatRomMt = {
	__index: CatRomMt,

	new: (points: {Point}, alpha: number?, tension: number?, loops: boolean?) -> CatRom,
	
	-- Piecewise methods
	GetSegmentAtTime: (self: CatRom, t: number) -> (Segment, number, number),
	PrecomputeUnitSpeedData: (
		self: CatRom,
		when: "now" | "on demand",
		strategy: "fast" | "accurate",
		degree: number?
	) -> (),
	SolveLength: (self: CatRom, from: number?, to: number?) -> number,
	SolveBulk: (
		self: CatRom,
		f: (segment: Segment, t: number) -> (),
		numSamples: number,
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> (),
	SolveBoundingBox: (self: CatRom) -> (Vector, Vector),
	CreateTween: (
		self: CatRom,
		tweenInfo: TweenInfo,
		callback: (segment: Segment, time: number) -> (),
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> Tween,

	-- Rotation-minimzing frame methods
	PrecomputeRMFs: (
		self: CatRom,
		numFramesPerSegment: number?,
		firstSegmentIndex: number?,
		lastSegmentIndex: number?
	) -> (),
	SolveCFrameRMF: (
		self: CatRom,
		t: number,
		unitSpeed: boolean?,
		prevFrame: CFrame?,
		numFramesPerSegment: number?
	) -> CFrame,
	Transport: (
		self: CatRom,
		data: Vector3 | CFrame,
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> Vector3 | CFrame,
	GetTransportInterpolant: (
		self: CatRom,
		data: Vector3 | CFrame,
		from: number?,
		to: number?,
		unitSpeed: boolean?
	) -> (t: number) -> Vector3 | CFrame,
	SlerpNormals: (
		self: CatRom,
		from: number,
		fromVector: Vector3,
		to: number,
		toVector: Vector3,
		t: number,
		unitSpeed: boolean?
	) -> Vector3,
	GetSlerpNormalsInterpolant: (
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
	SolveJerk:             (self: CatRom, t: number) -> Vector,
	SolveTangent:          (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveNormal:           (self: CatRom, t: number, unitSpeed: boolean?) -> Vector,
	SolveBinormal:         (self: CatRom, t: number, unitSpeed: boolean?) -> Vector3,
	SolveCurvature:        (self: CatRom, t: number, unitSpeed: boolean?) -> number,
	SolveTorsion:          (self: CatRom, t: number, unitSpeed: boolean?) -> number,
	SolveCFrameLookAlong:  (self: CatRom, t: number, unitSpeed: boolean?, upVector: Vector3?) -> CFrame,
	SolveCFrameFrenet:     (self: CatRom, t: number, unitSpeed: boolean?) -> CFrame,
	SolveCFrameSquad:      (self: CatRom, t: number, unitSpeed: boolean?) -> CFrame,
}

export type Segment = typeof(setmetatable(
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
	{} :: SegmentMt
))

export type SegmentMt = {
	__index: SegmentMt,

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
	) -> Segment,

	-- Basic methods
	SolvePosition:     (self: Segment, t: number) -> Vector,
	SolveVelocity:     (self: Segment, t: number) -> Vector,
	SolveAcceleration: (self: Segment, t: number) -> Vector,
	SolveJerk:         (self: Segment) -> Vector,
	SolveTangent:      (self: Segment, t: number) -> Vector,
	SolveNormal:       (self: Segment, t: number) -> Vector,
	SolveBinormal:     (self: Segment, t: number) -> Vector3,
	SolveCurvature:    (self: Segment, t: number) -> number,
	SolveTorsion:      (self: Segment, t: number) -> number,

	-- Moving frame methods
	SolveCFrameLookAlong: (self: Segment, t: number, upVector: Vector3?) -> CFrame,
	SolveCFrameFrenet:    (self: Segment, t: number) -> CFrame,
	SolveCFrameSquad:     (self: Segment, t: number) -> CFrame,
	SolveCFrameRMF:       (self: Segment, t: number, prevFrame: CFrame?) -> CFrame,
	PrecomputeRMFs: (self: Segment, numFrames: number, initialFrame: CFrame) -> (),
	
	-- Numerical methods
	SolveLength: (self: Segment, from: number?, to: number?) -> number,
	SolveBoundingBox: (self: Segment) -> (Vector, Vector),
	
	-- Arc length reparametrization methods
	Reparametrize: (self: Segment, s: number) -> number,
	PrecomputeUnitSpeedData: (self: Segment, precomputeNow: boolean, useChebAsLUT: boolean, degree: number) -> (),
	_ReparametrizeNewtonBisection: (self: Segment, s: number) -> number,
	_GetChebyshevInterpolant: (self: Segment, degree: number) -> Chebyshev,
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