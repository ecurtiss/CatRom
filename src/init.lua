local SplineFactory = require(script.SplineFactory)
local Types = require(script.Types)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_CHEB_DEGREE = 8
local DEFAULT_RMF_PRECOMPUTES = 4
local EPSILON = 2e-7

--[=[
	@class CatRom
	Manages chains of Catmull-Rom splines.
]=]
local CatRom = {}
CatRom.__index = CatRom

-- Removes adjacent points that are fuzzy-equal
local function getUniquePoints(points: {Types.Point}): {Types.Point}
	local prevPoint = points[1]
	local uniquePoints = {prevPoint}
	local i = 2

	for j = 2, #points do
		local point = points[j]
		if not point:FuzzyEq(prevPoint) then
			uniquePoints[i] = point
			i += 1
			prevPoint = point
		end
	end

	return uniquePoints
end

function CatRom.new(points: {Types.Point}, alpha: number?, tension: number?, loops: boolean?)
	alpha = alpha or DEFAULT_ALPHA -- Parametrization exponent
	tension = tension or DEFAULT_TENSION
	loops = not not loops

	-- Check types
	assert(type(points) == "table", "Points must be a table")
	assert(type(alpha) == "number", "Alpha must be a number")
	assert(type(tension) == "number", "Tension must be a number")
	assert(#points > 0, "Points table cannot be empty")
	
	local pointType = typeof(points[1])
	assert(pointType == "Vector2" or pointType == "Vector3" or pointType == "CFrame",
		"Points must be a table of Vector2s, Vector3s, or CFrames")
	for _, point in points do
		assert(typeof(point) == pointType, "All points must have the same type")
	end

	-- Get points
	points = getUniquePoints(points)

	if loops and not points[1]:FuzzyEq(points[#points]) then
		table.insert(points, points[1])
	end

	-- Early exits
	if #points <= 2 then
		local spline = if #points == 1
			then SplineFactory.CreatePointSpline(points[1], pointType)
			else SplineFactory.CreateLineSpline(points[1], points[2], pointType)

		return setmetatable({
			knots = {0, 1},
			length = spline.length,
			points = points,
			splines = {spline},
		}, CatRom)
	end

	-- Create splines
	local splines = SplineFactory.CreateSplines(points, alpha, tension, loops, pointType)
	local numSplines = #splines

	-- Tally length
	local totalLength = 0
	for _, spline in splines do
		totalLength += spline.length
	end

	-- Create knot vector
	local knots = table.create(numSplines + 1)
	knots[numSplines + 1] = 1

	local runningLength = 0
	for i, spline in splines do
		knots[i] = runningLength / totalLength
		runningLength += spline.length
	end

	return setmetatable({
		knots = knots,
		length = totalLength,
		points = points,
		splines = splines,
	}, CatRom)
end

--------------------------------------------------------------------------------
-- Piecewise methods -----------------------------------------------------------
--------------------------------------------------------------------------------

--[=[
	In CatRom, a spline S is a piecewise function of n interpolants defined on
	the interval [0, 1]. We partition [0, 1] into n subintervals of the form
	[k_i, k_{i+1}) where 0 = k1 < k2 < ... < kn < k_{n+1} = 1. The n+1 numbers
	k1, ..., k_{n+1} are called "knots".
	
	The i^th interpolant S_i is defined on the subinterval [k_i, k_{i+1}), where
	the width k_i - k_{i-1} is proportional to the arc length of S_i. That is,
	k_i - k_{i-1} is the arc length of S_i divided by the total arc length of S.
	
	Below is an example with n = 5. We can infer that S2 has a short arc length
	while S4 has a long arc length.

	k1               k2     k3           k4                       k5         k6
	|       S1       |  S2  |     S3     |           S4           |    S5    |
	0 ---------------------------------------------------------------------- 1

	To compute S(t), we find the subinterval [a, b) containing t and evaluate
	its corresponding interpolant S_j at t. However, in implementation, S_j is
	actually defined on [0, 1), so we first map [a, b) -> [0, 1). Hence
	S(t) = S_j((t - a) / (b - a)). Finally, for completeness, S(1) = S_n(1) with
	S_n having domain [0, 1] instead of [0, 1).

	Using the notation above, the below function GetSplineAtTime() takes as
	input t and returns three outputs:
	  1. the interpolant S_j
	  2. the time (t - a) / (b - a)
	  3. the index j.
]=]
function CatRom:GetSplineAtTime(t: number)
	assert(t >= 0 and t <= 1, "Time must be in [0, 1]")

	local splines = self.splines
	local knots = self.knots
	local numSplines = #splines

	-- Special cases
	if numSplines == 1 then
		return splines[1], t, 1
	elseif t == 0 then
		return splines[1], 0, 1
	elseif t == 1 then
		return splines[numSplines], 1, numSplines
	end

	-- Binary search for the spline containing t
	local left = 1
	local right = numSplines

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = knots[mid]

		if t >= intervalStart then
			local intervalEnd = knots[mid + 1]

			if t <= intervalEnd then
				local spline = splines[mid]
				local splineTime = (t - intervalStart) / (intervalEnd - intervalStart)
				return spline, splineTime, mid
			else
				left = mid + 1
			end
		else
			right = mid - 1
		end
	end

	-- This should never happen
	error("Failed to get spline")
end

function CatRom:PrecomputeUnitSpeedData(when: "now" | "on demand"?, strategy: "fast" | "accurate"?,  degree: number?)
	when = when or "now"
	strategy = strategy or "fast"
	degree = degree or DEFAULT_CHEB_DEGREE

	assert(when == "now" or when == "on demand", "when must be \"now\" or \"on demand\"")
	assert(strategy == "fast" or strategy == "accurate", "strategy must be \"fast\" or \"accurate\"")
	assert(type(degree) == "number" and degree >= 1 and degree % 2 == 0, "degree must be an integer >= 1")

	local precomputeNow = when == "now"
	local useChebAsLUT = strategy == "fast"

	for _, spline in self.splines do
		spline:PrecomputeUnitSpeedData(precomputeNow, useChebAsLUT, degree)
	end
end

function CatRom:SolveLength(from: number?, to: number?): number
	local a = from or 0
	local b = to or 1
	a, b = math.min(a, b), math.max(a, b)
	
	if a == 0 and b == 1 then
		return self.length
	end

	assert(a >= 0 and b <= 1, "Times must be in [0, 1]")
	
	local splineA, splineATime, splineAIndex = self:GetSplineAtTime(a)
	local splineB, splineBTime, splineBIndex = self:GetSplineAtTime(b)

	if splineAIndex == splineBIndex then
		return splineA:SolveLength(splineATime, splineBTime)
	end

	local lengthA = splineA:SolveLength(splineATime, 1)
	local lengthB = splineB:SolveLength(0, splineBTime)

	local intermediateLengths = 0
	for i = splineAIndex + 1, splineBIndex - 1 do
		intermediateLengths += self.splines[i].length
	end

	return lengthA + intermediateLengths + lengthB
end

function CatRom:SolveBulk(f: ({}, number) -> any, numSamples: number, from: number?, to: number?, unitSpeed: boolean?)
	local a = from or 0
	local b = to or 1
	assert(a <= b, "Time 'from' cannot be greater than time 'to'")
	assert(a >= 0 and b <= 1, "Times must be in [0, 1]")
	assert(type(numSamples) == "number", "Bad numSamples")
	
	numSamples = math.round(numSamples)
	if numSamples < 1 then
		return
	end

	local splines = self.splines
	local knots = self.knots

	local splineA, splineATime, splineAIndex = self:GetSplineAtTime(a)
	local splineB, splineBTime, splineBIndex = self:GetSplineAtTime(b)

	-- First sample
	f(splineA, splineATime)
	
	-- Samples 2, ..., numSamples - 1
	local lerpIncrement = (b - a) / (numSamples - 1)
	local previousDomainMax = knots[splineAIndex]
	local nextSampleTime = a + lerpIncrement
	local nextSampleIndex = 2

	for i = splineAIndex, splineBIndex do
		local spline = splines[i]
		local domainMin = previousDomainMax
		local domainMax = knots[i + 1]
		local domainWidth = domainMax - domainMin

		-- Run all samples in this spline
		while nextSampleTime <= domainMax and nextSampleIndex < numSamples do
			local t = (nextSampleTime - domainMin) / domainWidth
			f(spline, if unitSpeed then spline:Reparametrize(t) else t)
			nextSampleTime = a + nextSampleIndex * lerpIncrement
			nextSampleIndex += 1
		end

		if nextSampleIndex >= numSamples then
			break
		end

		previousDomainMax = domainMax
	end

	-- Last sample
	if numSamples > 1 then
		f(splineB, splineBTime)
	end
end

function CatRom:SolveBoundingBox(): (Types.Vector, Types.Vector)
	local splines = self.splines
	local n = #splines - 1

	local firstMin, firstMax = splines[1]:SolveBoundingBox()
	local minima = table.create(n)
	local maxima = table.create(n)

	for i = 1, n do
		minima[i], maxima[i] = splines[i + 1]:SolveBoundingBox()
	end

	local min = firstMin:Min(table.unpack(minima))
	local max = firstMax:Max(table.unpack(maxima))

	return min, max
end

--------------------------------------------------------------------------------
-- Rotation-minimizing frames --------------------------------------------------
--------------------------------------------------------------------------------

function CatRom:PrecomputeRotationMinimizingFrames(numFrames: number?, firstSplineIndex: number?, lastSplineIndex: number?)
	numFrames = if numFrames then math.max(1, numFrames) else DEFAULT_RMF_PRECOMPUTES
	firstSplineIndex = firstSplineIndex or 1
	lastSplineIndex = lastSplineIndex or #self.splines

	local prevFrame
	if firstSplineIndex == 1 then
		prevFrame = CFrame.lookAlong(self:SolvePosition(0), self:SolveTangent(0))
	else
		prevFrame = self.splines[firstSplineIndex - 1].rmfLUT[numFrames + 1]
	end

	for i = firstSplineIndex, lastSplineIndex do
		local spline = self.splines[i]
		spline:PrecomputeRotationMinimizingFrames(numFrames, prevFrame)
		prevFrame = spline.rmfLUT[numFrames + 1]
	end
end

--- If the user is tweening an RMF over time, then they can supply the RMF from
--- the previous frame. Otherwise, we precompute a lookup table of RMFs for only
--- the necessary splines.
function CatRom:SolveCFrame_RMF(t: number, unitSpeed: boolean?, prevFrame: CFrame?, numFrames: number?): CFrame
	local spline, splineTime, splineIndex = self:GetSplineAtTime(t)
	local splines = self.splines

	if prevFrame then
		return spline:SolveCFrame_RMF(if unitSpeed then spline:Reparametrize(splineTime) else splineTime, prevFrame)
	end

	-- Precompute only the necessary RMF LUTs
	if spline.rmfLUT == nil then
		if splines[1].rmfLUT == nil then
			self:PrecomputeRotationMinimizingFrames(numFrames, 1, splineIndex)
		else
			local i = splineIndex - 1
			while splines[i].rmfLUT == nil do
				i -= 1
			end
			self:PrecomputeRotationMinimizingFrames(numFrames, i + 1, splineIndex)
		end
	end

	return spline:SolveCFrame_RMF(if unitSpeed then spline:Reparametrize(splineTime) else splineTime, prevFrame)
end

function CatRom:GetParallelTransportInterpolant(data: Vector3 | CFrame, from: number?, to: number?, unitSpeed: boolean?): (number) -> Vector3 | CFrame
	from = from or 0
	to = to or 1
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")
	
	local dataType = typeof(data)
	local dataIsVector3 = dataType == "Vector3"
	assert(dataIsVector3 or dataType == "CFrame", "Bad data: Can only parallel transport Vector3s and CFrames")

	local initialFrame = self:SolveCFrame_RMF(from, unitSpeed)
	local totalTime = to - from

	if dataIsVector3 then
		local localVector = initialFrame:VectorToObjectSpace(data)
		return function(t: number): Vector3
			return self:SolveCFrame_RMF((t - from) / totalTime, unitSpeed):VectorToWorldSpace(localVector)
		end
	else
		local localCFrame = initialFrame:Inverse() * data
		return function(t: number): CFrame
			return self:SolveCFrame_RMF((t - from) / totalTime, unitSpeed) * localCFrame
		end
	end
end

function CatRom:GetNormalVectorInterpolant(from: number, fromVector: Vector3, to: number, toVector: Vector3, unitSpeed: boolean?): (number) -> Vector3
	assert(from >= 0 and from <= 1 and to >= 0 and to <= 1, "Times must be in [0, 1]")

	local totalTime = to - from

	local frameA = self:SolveCFrame_RMF(from, unitSpeed)
	local normalA = (fromVector - frameA.LookVector:Dot(fromVector) * frameA.LookVector).Unit
	local angleA = normalA:Angle(frameA.RightVector, frameA.LookVector)

	if normalA.Magnitude > EPSILON then
		normalA = normalA.Unit
	else
		error("fromVector cannot be tangent to the curve")
	end

	local frameB = self:SolveCFrame_RMF(to, unitSpeed)
	local normalB = (toVector - frameB.LookVector:Dot(toVector) * frameB.LookVector).Unit
	local angleB = normalB:Angle(frameB.RightVector, frameB.LookVector)

	if normalB.Magnitude > EPSILON then
		normalB = normalB.Unit
	else
		error("toVector cannot be tangent to the curve")
	end

	local delta = (angleB - angleA) % (2 * math.pi)
	if delta > math.pi then
		delta -= 2 * math.pi
	end

	return function(t: number): Vector3
		local frameT = self:SolveCFrame_RMF(from + t * totalTime, unitSpeed)
		local angleT = angleA + t * delta
		local normalT = math.cos(angleT) * frameT.RightVector + math.sin(angleT) * frameT.UpVector
		return normalT
	end
end

--------------------------------------------------------------------------------
-- Proxy methods ---------------------------------------------------------------
--------------------------------------------------------------------------------

function CatRom:SolvePosition(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolvePosition(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveVelocity(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveVelocity(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveAcceleration(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveAcceleration(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveJerk(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveJerk(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveTangent(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveTangent(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveNormal(t: number, unitSpeed: boolean?): Types.Vector
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveNormal(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveBinormal(t: number, unitSpeed: boolean?): Vector3
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveBinormal(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveCurvature(t: number, unitSpeed: boolean?): number
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCurvature(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveTorsion(t: number, unitSpeed: boolean?): number
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveTorsion(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveCFrame_LookAlong(t: number, unitSpeed: boolean?, upVector: Vector3?): CFrame
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCFrame_LookAlong(if unitSpeed then spline:Reparametrize(splineTime) else splineTime, upVector)
end

function CatRom:SolveCFrame_Frenet(t: number, unitSpeed: boolean?): CFrame
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCFrame_Frenet(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveCFrame_Squad(t: number, unitSpeed: boolean?): CFrame
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCFrame_Squad(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

return CatRom