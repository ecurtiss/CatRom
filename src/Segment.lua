--[[
	Notation
	t: Time parameter
	s: Arc length parameter
	r: Position
	T: Tangent vector
	N: Normal vector
	B: Binormal vector
	κ: Curvature
	τ: Torsion
]]

local Chebyshev = require(script.Parent.Chebyshev)
local Constants = require(script.Parent.Constants)
local GaussLegendre = require(script.Parent.GaussLegendre)
local Squad = require(script.Parent.Squad)
local Types = require(script.Parent.Types)
local Utils = require(script.Parent.Utils)

--[=[
	@class Segment

	A single segment of a Catmull-Rom spline that interpolates two control
	points. [CatRom:SolveBulk] and [CatRom:CreateTween] are the only methods
	that expose a [Segment].
]=]
local Segment: Types.SegmentMt = {} :: Types.SegmentMt
Segment.__index = Segment

function Segment.new(
	a: Types.Vector,
	b: Types.Vector,
	c: Types.Vector,
	d: Types.Vector,
	pointType: Types.PointType,
	q0: Types.Quaternion?,
	q1: Types.Quaternion?,
	q2: Types.Quaternion?,
	q3: Types.Quaternion?,
	length: number?
)
	local self = setmetatable({
		arcLengthCheb = nil,
		arcLengthChebDegree = nil,
		arcLengthChebIsLUT = nil,
		keyframeCheb = nil,
		length = length,
		rmfLUT = nil,
		type = pointType,

		a = a,
		b = b,
		c = c,
		d = d,

		q0 = q0,
		q1 = q1,
		q2 = q2,
		q3 = q3,
	}, Segment)

	if not length then
		self.length = self:SolveLength()
	end

	return self
end

--------------------------------------------------------------------------------
-- Basic methods ---------------------------------------------------------------
--------------------------------------------------------------------------------

function Segment:SolvePosition(t: number): Types.Vector
	-- r(t) using Horner's method
	return self.d + t * (self.c + t * (self.b + t * self.a))
end

function Segment:SolveVelocity(t: number): Types.Vector
	-- r'(t) using Horner's method
	return self.c + t * (2 * self.b + t * 3 * self.a)
end

function Segment:SolveAcceleration(t: number): Types.Vector
	-- r''(t)
	return 6 * self.a * t + 2 * self.b
end

function Segment:SolveJerk(): Types.Vector
	-- r'''(t)
	return 6 * self.a
end

function Segment:SolveTangent(t: number): Types.Vector
	-- T(t) = r'(t) / |r'(t)|
	return Utils.Unit(self:SolveVelocity(t))
end

function Segment:SolveNormal(t: number): Types.Vector
	assert(self.type ~= "number", "SolveNormal is undefined on number splines")

	if self.type == "Vector2" then
		local tangent = self:SolveTangent(t)
		return Vector2.new(-tangent.Y, tangent.X)
	else
		-- N(t) = T'(t) / |T'(t)|
		-- The return is equivalent to N(t) when the derivatives are carried out.
		-- In particular, the vector being normalized is T'(t) * |r'(t)| ^ 3, but
		-- the |r'(t)| ^ 3 scaling doesn't affect the result because we normalize
		-- it anyway. This scaled version is faster to compute.
		local vel = self:SolveVelocity(t)
		local acc = self:SolveAcceleration(t)
		return (acc * vel.Magnitude ^ 2 - vel * vel:Dot(acc)).Unit
	end
end

function Segment:SolveBinormal(t: number): Vector3
	assert(self.type == "Vector3" or self.type == "CFrame", "SolveBinormal is only defined on Vector3 and CFrame splines")
	-- T(t) x N(t)
	return self:SolveTangent(t):Cross(self:SolveNormal(t))
end

function Segment:SolveCurvature(t: number): number
	assert(self.type ~= "number", "SolveCurvature is undefined on number splines")

	if self.type == "Vector2" then
		local vel = self:SolveVelocity(t)
		local acc = self:SolveAcceleration(t)
		return vel:Cross(acc) / vel.Magnitude^3
	else
		local vel = self:SolveVelocity(t)
		local acc = self:SolveAcceleration(t)
		local speed = vel.Magnitude
		local dTangent = acc / speed - vel * vel:Dot(acc) / speed ^ 3
	
		-- κ(t) = |T'(t)| / |r'(t)|
		return dTangent.Magnitude / speed
	end
end

function Segment:SolveTorsion(t: number): number
	assert(self.type == "Vector3" or self.type == "CFrame", "SolveTorsion is only defined on Vector3 and CFrame splines")

	local vel = self:SolveVelocity(t)
	local acc = self:SolveAcceleration(t)
	local jerk = self:SolveJerk()
	local cross = vel:Cross(acc)

	-- τ = ((r' x r'') • r''') / |r' x r''|^2
	return cross:Dot(jerk) / cross.Magnitude^2
end

--------------------------------------------------------------------------------
-- Moving frame methods --------------------------------------------------------
--------------------------------------------------------------------------------

function Segment:SolveCFrameLookAlong(t: number, upVector: Vector3?): CFrame
	assert(self.type == "Vector3" or self.type == "CFrame", "SolveCFrameLookAlong is only defined on Vector3 and CFrame splines")

	local pos = self:SolvePosition(t)
	
	if self.length == 0 then -- Spline is a point
		return CFrame.new(pos)
	else
		return CFrame.lookAlong(pos, self:SolveVelocity(t), upVector or Vector3.yAxis)
	end
end

function Segment:SolveCFrameFrenet(t: number): CFrame
	assert(self.type == "Vector3" or self.type == "CFrame", "SolveCFrameFrenet is only defined on Vector3 and CFrame splines")

	local pos = self:SolvePosition(t)
	local tangent = self:SolveTangent(t)
	local normal = self:SolveNormal(t)
	local binormal = tangent:Cross(normal)

	return CFrame.fromMatrix(pos, binormal, normal)
end

function Segment:SolveCFrameSquad(t: number): CFrame
	assert(self.type == "CFrame", "SolveCFrameSquad is only defined on CFrame splines")

	local pos = self:SolvePosition(t)
	local q0 = self.q0
	local q1 = self.q1
	
	if q1 then
		local qw, qx, qy, qz = Squad(q0, q1, self.q2, self.q3, t)
		return CFrame.new(pos.X, pos.Y, pos.Z, qx, qy, qz, qw)
	else -- Spline is a point
		return CFrame.new(pos.X, pos.Y, pos.Z, q0[2], q0[3], q0[4], q0[1])
	end
end

local function doubleReflect(
	prevPos: Vector3,
	prevRight: Vector3,
	prevUp: Vector3,
	prevLook: Vector3,
	pos: Vector3,
	look: Vector3
): (Vector3, Vector3, Vector3, Vector3, CFrame)
	-- First reflection
	local planeNormal1 = (pos - prevPos).Unit
	local prevRightReflected = prevRight - 2 * planeNormal1:Dot(prevRight) * planeNormal1
	local prevLookReflected = prevLook - 2 * planeNormal1:Dot(prevLook) * planeNormal1
	local prevUpReflected = prevUp - 2 * planeNormal1:Dot(prevUp) * planeNormal1

	-- Second reflection
	local planeNormal2 = (look - prevLookReflected).Unit
	local right = prevRightReflected - 2 * planeNormal2:Dot(prevRightReflected) * planeNormal2
	local up = prevUpReflected - 2 * planeNormal2:Dot(prevUpReflected) * planeNormal2

	return pos, right, up, look, CFrame.fromMatrix(pos, right, up, -look)
end

-- Uses the double reflection method to precompute rotation-minimizing frames
-- Source: Wang, "Computation of Rotation Minimizing Frames" (2008)
function Segment:PrecomputeRMFs(numFrames: number, initialFrame: CFrame)
	local rmfLUT = table.create(numFrames + 1)
	rmfLUT[1] = initialFrame

	local prevPos = initialFrame.Position
	local prevRight = initialFrame.RightVector
	local prevUp = initialFrame.UpVector
	local prevLook = initialFrame.LookVector

	for i = 1, numFrames do
		local t = i / numFrames
		prevPos, prevRight, prevUp, prevLook, rmfLUT[i + 1] = doubleReflect(
			prevPos,
			prevRight,
			prevUp,
			prevLook,
			self:SolvePosition(t),
			self:SolveTangent(t))
	end

	self.rmfLUT = rmfLUT
end

function Segment:SolveCFrameRMF(t: number, prevFrame: CFrame?): CFrame
	assert(self.type == "Vector3" or self.type == "CFrame", "SolveCFrameRMF is only defined on Vector3 and CFrame splines")
	assert(prevFrame or self.rmfLUT, "Must call PrecomputeRMFs before using SolveCFrameRMF")

	if self.length == 0 then -- Spline is a point
		return prevFrame or self.rmfLUT[1]
	end

	if not prevFrame then
		local prevFrameIndex = math.floor(t * (#self.rmfLUT - 1)) + 1
		local prevFrameTime = (prevFrameIndex - 1) / (#self.rmfLUT - 1)
		prevFrame = self.rmfLUT[prevFrameIndex]

		if t - prevFrameTime < Constants.MACHINE_EPSILON then
			return prevFrame
		end
	end

	local pos = self:SolvePosition(t)
	local tangent = self:SolveTangent(t)

	if pos:FuzzyEq(prevFrame.Position, Constants.MACHINE_EPSILON) then
		local prevTangent = prevFrame.LookVector

		if tangent:FuzzyEq(prevTangent, Constants.MACHINE_EPSILON) then
			return prevFrame
		else
			local axis = prevTangent:Cross(tangent)
			local angle = prevTangent:Angle(tangent)
			return CFrame.fromAxisAngle(axis, angle) * prevFrame.Rotation + pos
		end
	end

	local _, _, _, _, cf = doubleReflect(
		prevFrame.Position,
		prevFrame.RightVector,
		prevFrame.UpVector,
		prevFrame.LookVector,
		self:SolvePosition(t),
		self:SolveTangent(t))

	return cf
end

--------------------------------------------------------------------------------
-- Numerical methods -----------------------------------------------------------
--------------------------------------------------------------------------------

local function getLengthIntegrand(segment: Types.Segment): (number) -> number
	if segment.type == "number" then
		return function(x: number)
			return math.abs(segment:SolveVelocity(x))
		end
	else
		return function(x: number)
			return segment:SolveVelocity(x).Magnitude
		end
	end
end

function Segment:SolveLength(from: number?, to: number?): number
	local a = from or 0
	local b = to or 1
	a, b = math.min(a, b), math.max(a, b)

	if a == 0 and b == 1 and self.length then
		return self.length
	else
		return GaussLegendre.Ten(getLengthIntegrand(self), a, b)
	end
end

local function addBoundingBoxCandidate(a, b, c, candidates, segment)
	local t1, t2 = Utils.SolveQuadratic(a, b, c)

	if t1 ~= nil then
		if t1 >= 0 and t1 <= 1 then
			table.insert(candidates, segment:SolvePosition(t1))
		end
		if t2 ~= nil and t2 >= 0 and t2 <= 1 then
			table.insert(candidates, segment:SolvePosition(t2))
		end
	end
end

function Segment:SolveBoundingBox(): (Types.Vector, Types.Vector)
	-- First derivative coefficients
	local a1 = 3 * self.a
	local b1 = 2 * self.b
	local c1 = self.c

	local candidates = { self:SolvePosition(0), self:SolvePosition(1) }

	if self.type == "number" then
		addBoundingBoxCandidate(a1, b1, c1, candidates, self)
	elseif self.type == "Vector2" then
		addBoundingBoxCandidate(a1.X, b1.X, c1.X, candidates, self)
		addBoundingBoxCandidate(a1.Y, b1.Y, c1.Y, candidates, self)
	else
		addBoundingBoxCandidate(a1.X, b1.X, c1.X, candidates, self)
		addBoundingBoxCandidate(a1.Y, b1.Y, c1.Y, candidates, self)
		addBoundingBoxCandidate(a1.Z, b1.Z, c1.Z, candidates, self)
	end

	return Utils.Min(candidates), Utils.Max(candidates)
end

--------------------------------------------------------------------------------
-- Arc length reparametrization ------------------------------------------------
--------------------------------------------------------------------------------

function Segment:Reparametrize(t: number, unitSpeed: boolean?): number
	if self.keyframeCheb then
		return self:ReparametrizeAsKeyframeAnimation(t)
	elseif unitSpeed then
		return self:ReparametrizeByArcLength(t)
	else
		return t
	end
end

-- Reparametrizes s in terms of arc length, i.e., returns the input t that
-- yields the point s of the way along the segment.
function Segment:ReparametrizeByArcLength(s: number): number
	assert(s >= 0 and s <= 1, "Time must be in [0, 1]")

	if s == 0 or s == 1 then
		return s
	elseif self.length == 0 then
		return 0
	end

	if self.arcLengthChebIsLUT ~= nil then
		-- arcLengthChebIsLUT ~= nil is a proxy to check whether the user called
		-- PrecomputeUnitSpeedData

		if not self.arcLengthCheb then
			-- User precomputed with "on demand" preference
			self.arcLengthCheb = self:_GetInvertedArcLengthCheb(self.arcLengthChebDegree)
		end

		if self.arcLengthChebIsLUT then
			-- Use the cheb's grid values as a lookup table
			local grid = self.arcLengthCheb.grid
			local gridValues = self.arcLengthCheb.gridValues
			local leftBound = grid[1]
		
			-- TODO: Make this a binary search
			for i = 2, #grid do
				local rightBound = grid[i]
		
				if s >= leftBound and s <= rightBound then
					local t = (s - leftBound) / (rightBound - leftBound)
					return gridValues[i - 1] * (1 - t) + gridValues[i] * t
				end
		
				leftBound = rightBound
			end

			warn("\"fast\" reparametrization strategy failed; reverting to \"accurate\" strategy")
		end

		return self.arcLengthCheb:Evaluate(s)
	else
		return self:_ReparametrizeByArcLengthNewtonBisection(s)
	end
end

-- Performs the actual arc length reparametrization
-- s = (1/L) * \int_{0}^{t} |r'(u)|du = (1/L) * (F(t) - F(0)) = F(t)/L
-- where t is solved as the root-finding problem f(t) = F(t)/L - s = 0.
function Segment:_ReparametrizeByArcLengthNewtonBisection(s: number): number
	local length = self.length

	if s == 0 or s == 1 then
		return s
	elseif length == 0 then
		return 0
	end

	-- Hybrid of Newton's method and bisection
	-- https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
	local integrand = getLengthIntegrand(self)
	local t = s
	local lower = 0
	local upper = 1

	for _ = 1, Constants.MAX_NEWTON_ITERATIONS do
		-- It is mathematically equivalent to instead use
		-- f = GaussLegendre.Ten(integrand, 0, t) - s * length
		-- g = self:SolveVelocity(t).Magnitude
		-- This has the benefit of being able to precompute s * length.
		-- However, in practice, it converges slower.
		local f =  GaussLegendre.Ten(integrand, 0, t) / length - s
		if math.abs(f) < Constants.MACHINE_EPSILON then
			return t
		end

		-- Compute next candidate via Newton's method
		local g = Utils.Magnitude(self:SolveVelocity(t)) / length
		local candidate = t - f / g

		if f > 0 then
			-- Solution is below t; decrease upper bound
			upper = t
			-- Try next candidate, or use midpoint of bounds if the candidate
			-- is out of bounds (assumes candidate < t)
			t = candidate <= lower and (upper + lower) / 2 or candidate
		else
			-- Solution is above t; increase lower bound
			lower = t
			-- Try next candidate, or use midpoint of bounds if the candidate
			-- is out of bounds (assumes candidate > t)
			t = candidate >= upper and (upper + lower) / 2 or candidate
		end
	end

	warn("Reached maximum Newton iterations")
	return (lower + upper) / 2
end

function Segment:PrecomputeUnitSpeedData(precomputeNow: boolean, useArcLengthChebAsLUT: boolean, degree: number)
	if self.length == 0 then
		return
	end

	self.arcLengthChebIsLUT = useArcLengthChebAsLUT

	if precomputeNow then
		self.arcLengthCheb = self:_GetInvertedArcLengthCheb(degree)
	else
		-- Save the degree for when we create the chebyshev interpolant later
		self.arcLengthChebDegree = degree
	end
end

function Segment:_GetInvertedArcLengthCheb(degree: number)
	local integrand = getLengthIntegrand(self)
	local length = self.length
	local arcLengthCheb = Chebyshev.new(function(t)
		return GaussLegendre.Ten(integrand, 0, t) / length
	end, degree)

	return arcLengthCheb:Invert()
end

function Segment:ReparametrizeAsKeyframeAnimation(t: number): number
	assert(t >= 0 and t <= 1, "Time must be in [0, 1]")

	if t == 0 or t == 1 then
		return t
	elseif self.length == 0 then
		return 0
	end

	return self.keyframeCheb:Evaluate(t)
end

function Segment:_SetKeyframeSegment(keyframeSegment: Types.Segment)
	local min = keyframeSegment:SolvePosition(0)
	local width = keyframeSegment:SolvePosition(1) - min

	-- This cheb is assumed to be invertible because
	-- (1) the keyframes are monotone increasing
	-- (2) the centripetal parametrization guarantees no self-intersections
	self.keyframeCheb = Chebyshev.new(function(t)
		return (keyframeSegment:SolvePosition(t) - min) / width
	end, 3):Invert()
end

return Segment