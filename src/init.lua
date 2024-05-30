-- A package for creating chains of Catmull-Rom splines
-- By AstroCode (https://github.com/ecurtiss/catrom)

local Spline = require(script.Spline)
local Utils = require(script.Utils)
local Types = require(script.Types)

local toTransform = Utils.ToTransform

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_REPARAMETRIZATION_PRECOMPUTES = 16
local DEFAULT_RMF_PRECOMPUTES = 16

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

	points = getUniquePoints(points)
	local numPoints = #points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]

	-- Early exits
	if numPoints == 1 then
		return setmetatable({
			knots = {0, 1},
			length = 0,
			loops = loops,
			points = points,
			splines = {Spline.fromPoint(points[1], pointType)},
		}, CatRom)
	elseif numPoints == 2 then
		local spline = Spline.fromLine(firstPoint, lastPoint, pointType)
		return setmetatable({
			knots = {0, 1},
			length = spline.length,
			loops = loops,
			points = points,
			splines = {spline},
		}, CatRom)
	end

	-- Extrapolate to get 0th and n+1th points
	local zerothPoint, veryLastPoint
	if loops then
		if not firstPoint:FuzzyEq(lastPoint) then
			table.insert(points, firstPoint)
			numPoints += 1
		end
		zerothPoint = points[numPoints - 1]
		veryLastPoint = points[2]
	else
		zerothPoint = points[2]:Lerp(firstPoint, 2)
		veryLastPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
	end

	-- Create splines
	local numSplines = numPoints - 1
	local splines = table.create(numSplines)
	local totalLength = 0

	-- Sliding window of control points to quicken instantiating splines
	local window1 = toTransform(zerothPoint, pointType) -- FIX: Don't pack them in tables instead?
	local window2 = toTransform(firstPoint, pointType)
	local window3 = toTransform(points[2], pointType)
	local window4 = toTransform(points[3] or veryLastPoint, pointType)

	-- Add the first spline, whose first point is not in the points table.
	splines[1] = Spline.new(window1, window2, window3, window4, alpha, tension)
	totalLength += splines[1].length

	-- Add the middle splines
	for i = 1, numPoints - 3 do
		-- Shift the window
		window1 = window2
		window2 = window3
		window3 = window4
		window4 = toTransform(points[i + 3], pointType)

		local spline = Spline.new(window1, window2, window3, window4, alpha, tension)
		totalLength += spline.length
		splines[i + 1] = spline
	end

	-- Add the last spline, whose fourth point is not in the points table
	splines[numSplines] = Spline.new(window2, window3, window4,
		toTransform(veryLastPoint, pointType), alpha, tension)
	totalLength += splines[numSplines].length

	-- Get the knots of the spline
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
		loops = loops,
		points = points,
		splines = splines,
	}, CatRom)
end

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

function CatRom:PrecomputeArcLengthParams(numIntervals: number?)
	numIntervals = if numIntervals then math.max(1, math.round(numIntervals)) else DEFAULT_REPARAMETRIZATION_PRECOMPUTES
	for _, spline in self.splines do
		spline:PrecomputeArcLengthParams(numIntervals)
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

function CatRom:SolveBulk(f: ({}, number) -> any, numSamples: number, from: number?, to: number?--[[, unitSpeed: boolean?]])
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

		-- Gather all samples in this spline
		local times = {}
		while nextSampleTime <= domainMax and nextSampleIndex < numSamples do
			table.insert(times, (nextSampleTime - domainMin) / domainWidth)
			nextSampleTime = a + nextSampleIndex * lerpIncrement
			nextSampleIndex += 1
		end

		previousDomainMax = domainMax

		if #times == 0 then
			continue
		end

		-- times = if unitSpeed then spline:ReparametrizeBulk(times) else times
		for _, t in times do
			f(spline, t)
		end

		if nextSampleIndex >= numSamples then
			break
		end
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
			self:PrecomputeRotationMinimizingFrames(numFrames, i + 1)
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

	local frameB = self:SolveCFrame_RMF(to, unitSpeed)
	local normalB = (toVector - frameB.LookVector:Dot(toVector) * frameB.LookVector).Unit
	local angleB = normalB:Angle(frameB.RightVector, frameB.LookVector)

	return function(t: number): Vector3
		local frameT = self:SolveCFrame_RMF(from + t * totalTime, unitSpeed)
		local angleT = angleA * (1 - t) + angleB * t
		local normalT = math.cos(angleT) * frameT.RightVector + math.sin(angleT) * frameT.UpVector
		return normalT
	end
end

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