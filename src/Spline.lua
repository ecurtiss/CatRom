--[[
	Catmull-Rom spline class

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
local GaussLegendre = require(script.Parent.GaussLegendre)
local Squad = require(script.Parent.Squad)
local Utils = require(script.Parent.Utils)
local Types = require(script.Parent.Types)

local MAX_NEWTON_ITERATIONS = 16
local EPSILON = 2e-7

--[=[
	@class Spline

	Catmull-Rom spline class
]=]
local Spline = {}
Spline.__index = Spline

function Spline.new(trans0, trans1, trans2, trans3, alpha, tension)
	-- Parametrize
	-- https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
	local pos0 = trans0[1]
	local pos1 = trans1[1]
	local pos2 = trans2[1]
	local pos3 = trans3[1]

	local pos1_pos0 = pos1 - pos0
	local pos2_pos1 = pos2 - pos1
	local pos3_pos2 = pos3 - pos2

	local a, b, c
	local t0 = 0
	local t1 = pos1_pos0.Magnitude ^ alpha + t0
	local t2 = pos2_pos1.Magnitude ^ alpha + t1
	local t3 = pos3_pos2.Magnitude ^ alpha + t2

	local scalar = (1 - tension) * (t2 - t1)

	local m1 = scalar * (pos1_pos0 / (t1 - t0) - (pos2 - pos0) / (t2 - t0) + pos2_pos1 / (t2 - t1))
	local m2 = scalar * (pos2_pos1 / (t2 - t1) - (pos3 - pos1) / (t3 - t1) + pos3_pos2 / (t3 - t2))

	a = 2 * -pos2_pos1 + m1 + m2
	b = 3 * pos2_pos1 - 2 * m1 - m2
	c = m1
	
	local self = setmetatable({
		cheb = nil,
		chebDegree = nil,
		chebIsLUT = nil,
		length = nil,
		rmfLUT = nil,
		type = if trans0[2] then "CFrame" else typeof(pos0),

		-- Rotations (nil if type is Vector2 or Vector3)
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],

		-- Coefficient vectors for position/velocity/acceleration polynomials
		a = a,
		b = b,
		c = c,
		d = pos1,
	}, Spline)
	self.length = self:SolveLength()
	
	return self
end

function Spline.fromPoint(point: Types.Point, pointType: Types.PointType)
	local trans = Utils.ToTransform(point, pointType)
	local pos = trans[1]
	local zero = pos * 0

	return setmetatable({
		cheb = nil,
		chebDegree = nil,
		chebIsLUT = nil,
		length = 0,
		rmfLUT = nil,
		type = pointType,

		rot0 = trans[2],

		a = zero,
		b = zero,
		c = zero,
		d = pos,
	}, Spline)
end

function Spline.fromLine(p1: Types.Point, p2: Types.Point, pointType: Types.PointType)
	local trans0 = Utils.ToTransform(p2:Lerp(p1, 2), pointType)
	local trans1 = Utils.ToTransform(p1, pointType)
	local trans2 = Utils.ToTransform(p2, pointType)
	local trans3 = Utils.ToTransform(p1:Lerp(p2, 2), pointType)

	local pos1 = trans1[1]
	local zero = pos1 * 0
	local pos2_pos1 = trans2[1] - pos1

	return setmetatable({
		cheb = nil,
		chebDegree = nil,
		chebIsLUT = nil,
		length = pos2_pos1.Magnitude,
		rmfLUT = nil,
		type = if trans0[2] then "CFrame" else typeof(pos1),
		
		rot0 = trans0[2],
		rot1 = trans1[2],
		rot2 = trans2[2],
		rot3 = trans3[2],

		a = zero,
		b = zero,
		c = pos2_pos1,
		d = pos1,
	}, Spline)
end

--------------------------------------------------------------------------------
-- Basic methods ---------------------------------------------------------------
--------------------------------------------------------------------------------

function Spline:SolvePosition(t: number): Types.Vector
	-- r(t) using Horner's method
	return self.d + t * (self.c + t * (self.b + t * self.a))
end

function Spline:SolveVelocity(t: number): Types.Vector
	-- r'(t) using Horner's method
	return self.c + t * (2 * self.b + t * 3 * self.a)
end

function Spline:SolveAcceleration(t: number): Types.Vector
	-- r''(t)
	return 6 * self.a * t + 2 * self.b
end

function Spline:SolveJerk(): Types.Vector
	-- r'''(t)
	return 6 * self.a
end

function Spline:SolveTangent(t: number): Types.Vector
	-- T(t) = r'(t) / |r'(t)|
	return self:SolveVelocity(t).Unit
end

function Spline:SolveNormal(t: number): Types.Vector
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

function Spline:SolveBinormal(t: number): Vector3
	assert(self.type ~= "Vector2", "SolveBinormal is undefined on Vector2 splines")
	-- T(t) x N(t)
	return self:SolveTangent(t):Cross(self:SolveNormal(t))
end

function Spline:SolveCurvature(t: number): number
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

function Spline:SolveTorsion(t: number): number
	assert(self.type ~= "Vector2", "SolveTorsion is undefined on Vector2 splines")

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

local function solveCFrameForPointSpline(pos: Vector3, rot: Types.Quaternion): CFrame
	if rot then
		return CFrame.new(pos.X, pos.Y, pos.Z, rot[2], rot[3], rot[4], rot[1])
	else
		return CFrame.new(pos)
	end
end

function Spline:SolveCFrame_LookAlong(t: number, upVector: Vector3?): CFrame
	assert(self.type ~= "Vector2", "SolveCFrame_LookAlong is undefined on Vector2 splines")

	local pos = self:SolvePosition(t)
	local tangent = self:SolveVelocity(t)

	if tangent.Magnitude == 0 then -- Spline is a point
		return solveCFrameForPointSpline(pos, self.rot0)
	else
		return CFrame.lookAlong(pos, tangent, upVector or Vector3.yAxis)
	end
end
--- Returns a CFrame with the LookVector, UpVector, and RightVector being the
--- tangent, normal, and binormal vectors respectively.
function Spline:SolveCFrame_Frenet(t: number): CFrame
	assert(self.type ~= "Vector2", "SolveCFrame_Frenet is undefined on Vector2 splines")

	local pos = self:SolvePosition(t)
	local tangent = self:SolveTangent(t)

	if tangent.Magnitude == 0 then -- Spline is a point
		return solveCFrameForPointSpline(pos, self.rot0)
	else
		local normal = self:SolveNormal(t)
		local binormal = tangent:Cross(normal)
		return CFrame.fromMatrix(pos, binormal, normal)
	end
end

function Spline:SolveCFrame_Squad(t: number): CFrame
	assert(self.type == "CFrame", "SolveCFrame_Squad is only defined on CFrame splines")

	local pos = self:SolvePosition(t)
	local rot0 = self.rot0
	local rot1 = self.rot1
	
	if rot1 then
		local qw, qx, qy, qz = Squad(rot0, rot1, self.rot2, self.rot3, t)
		return CFrame.new(pos.X, pos.Y, pos.Z, qx, qy, qz, qw)
	else -- Spline is a point
		return solveCFrameForPointSpline(pos, rot0)
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

--- Uses the double reflection method to precompute rotation-minimizing frames
--- Source: Wang, "Computation of Rotation Minimizing Frames" (2008)
function Spline:PrecomputeRotationMinimizingFrames(numFrames: number, initialFrame: CFrame)
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

function Spline:SolveCFrame_RMF(t: number, prevFrame: CFrame?): CFrame
	assert(self.type ~= "Vector2", "SolveCFrame_RMF is undefined on Vector2 splines")
	assert(prevFrame or self.rmfLUT, "Must call PrecomputeRotationMinimizingFrames before using SolveCFrame_RMF")

	if not prevFrame then
		local prevFrameIndex = math.floor(t * (#self.rmfLUT - 1)) + 1
		local prevFrameTime = (prevFrameIndex - 1) / (#self.rmfLUT - 1)
		prevFrame = self.rmfLUT[prevFrameIndex]

		if t - prevFrameTime < EPSILON then
			return prevFrame
		end
	end

	local pos = self:SolvePosition(t)
	local tangent = self:SolveTangent(t)

	if pos:FuzzyEq(prevFrame.Position) then
		if tangent:FuzzyEq(prevFrame.LookVector) then
			return prevFrame
		else
			error("prevFrame too close to new frame")
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

function Spline:SolveLength(a: number?, b: number?): number
	a = a or 0
	b = b or 1

	if a == 0 and b == 1 and self.length then
		return self.length
	end

	return GaussLegendre.Ten(function(x)
		return self:SolveVelocity(x).Magnitude
	end, a, b)
end

local function addBoundingBoxCandidate(a, b, c, candidates, spline)
	local t1, t2 = Utils.SolveQuadratic(a, b, c)

	if t1 ~= nil then
		if t1 >= 0 and t1 <= 1 then
			table.insert(candidates, spline:SolvePosition(t1))
		end
		if t2 ~= nil and t2 >= 0 and t2 <= 1 then
			table.insert(candidates, spline:SolvePosition(t2))
		end
	end
end

function Spline:SolveBoundingBox(): (Types.Vector, Types.Vector)
	-- First derivative coefficients
	local a1 = 3 * self.a
	local b1 = 2 * self.b
	local c1 = self.c

	local candidates = { self:SolvePosition(0) }
	addBoundingBoxCandidate(a1.X, b1.X, c1.X, candidates, self)
	addBoundingBoxCandidate(a1.Y, b1.Y, c1.Y, candidates, self)
	if self.type ~= "Vector2" then
		addBoundingBoxCandidate(a1.Z, b1.Z, c1.Z, candidates, self)
	end

	local pos = self:SolvePosition(1)
	local min = pos:Min(table.unpack(candidates))
	local max = pos:Max(table.unpack(candidates))

	return min, max
end

--------------------------------------------------------------------------------
-- Arc length reparametrization ------------------------------------------------
--------------------------------------------------------------------------------

-- Reparametrizes s in terms of arc length, i.e., returns the input t that
-- yields the point s of the way along the spline.
function Spline:Reparametrize(s: number): number
	assert(s >= 0 and s <= 1, "Time must be in [0, 1]")

	if s == 0 or s == 1 then
		return s
	elseif self.length == 0 then
		return 0
	end

	if self.chebIsLUT ~= nil then
		-- chebIsLUT ~= nil is a proxy to check whether the user called
		-- PrecomputeUnitSpeedData

		if not self.cheb then
			-- User precomputed with "on demand" preference
			self.cheb = self:_GetChebyshevInterpolant(self.chebDegree)
		end

		if self.chebIsLUT then
			-- Use the cheb's grid values as a lookup table
			local grid = self.cheb.grid
			local gridValues = self.cheb.gridValues
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

		return self.cheb:Evaluate(s)
	else
		return self:_ReparametrizeNewtonBisection(s)
	end
end

-- Performs the actual arc length reparametrization
-- s = (1/L) * \int_{0}^{t} |r'(u)|du = (1/L) * (F(t) - F(0)) = F(t)/L
-- where t is solved as the root-finding problem f(t) = F(t)/L - s = 0.
function Spline:_ReparametrizeNewtonBisection(s: number): number
	local length = self.length

	if s == 0 or s == 1 then
		return s
	elseif length == 0 then
		return 0
	end

	-- Hybrid of Newton's method and bisection
	-- https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
	local integrand = function(x)
		return self:SolveVelocity(x).Magnitude
	end

	local t = s
	local lower = 0
	local upper = 1

	for _ = 1, MAX_NEWTON_ITERATIONS do
		-- It is mathematically equivalent to instead use
		-- f = GaussLegendre.Ten(integrand, 0, t) - s * length
		-- g = self:SolveVelocity(t).Magnitude
		-- This has the benefit of being able to precompute s * length.
		-- However, in practice, it converges slower.
		local f =  GaussLegendre.Ten(integrand, 0, t) / length - s
		if math.abs(f) < EPSILON then
			return t
		end

		-- Compute next candidate via Newton's method
		local g = self:SolveVelocity(t).Magnitude / length
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

	warn("Exceeded maximum Newton iterations")
	return (lower + upper) / 2
end

function Spline:PrecomputeUnitSpeedData(precomputeNow: boolean, useChebAsLUT: boolean, degree: number)
	self.chebIsLUT = useChebAsLUT

	if precomputeNow then
		self.cheb = self:_GetChebyshevInterpolant(degree)
	else
		-- Save the degree for when we create the chebyshev interpolant later
		self.chebDegree = degree
	end
end

function Spline:_GetChebyshevInterpolant(degree: number)
	local interpolant = function(u)
		return self:SolveVelocity(u).Magnitude
	end
	local length = self.length
	local cheb = Chebyshev.new(function(t)
		return GaussLegendre.Ten(interpolant, 0, t) / length
	end, degree)

	return cheb:Invert()
end

return Spline