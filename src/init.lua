local Spline = require(script.Spline)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_PRECOMPUTE_INTERVALS = 16

type Point = CFrame | Vector2 | Vector3

local CatRom = {}
CatRom.__index = CatRom

local function CFrameToQuaternion(cframe)
	local axis, angle = cframe:ToAxisAngle()
	angle /= 2
	axis = math.sin(angle) * axis
	return {math.cos(angle), axis.X, axis.Y, axis.Z}
end

local function ToTransform(point, pointType)
	if pointType == "Vector2" or pointType == "Vector3" then
		return {point}
	elseif pointType == "CFrame" then
		return {point.Position, CFrameToQuaternion(point)}
	end

	error("Bad inputs")
end

function CatRom.new(points: {Point}, alpha: number?, tension: number?)
	alpha = alpha or DEFAULT_ALPHA -- Parameterization exponent
	tension = tension or DEFAULT_TENSION

	-- Type check
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

	-- Remove equal adjacent points
	local uniquePoints = {} do
		local prevPoint = points[1]
		uniquePoints[1] = prevPoint
		local i = 2
		for j = 2, #points do
			local point = points[j]
			if point:FuzzyEq(prevPoint) then
				uniquePoints[i] = point
				i += 1
				prevPoint = point
			end
		end
	end
	points = uniquePoints
	local numPoints = #points

	-- Early exit: 1 point
	if numPoints == 1 then
		return setmetatable({
			alpha = alpha,
			tension = tension,

			splines = {Spline.fromPoint(ToTransform(points[1], pointType))},
			domains = {0},

			length = 0
		}, CatRom)
	end

	-- Extrapolate to get 0th and n+1th points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	local zerothPoint, veryLastPoint do
		if firstPoint:FuzzyEq(lastPoint) then
			-- Loops
			zerothPoint = points[numPoints - 1]
			veryLastPoint = points[2]
		else
			-- Does not loop
			zerothPoint = points[2]:Lerp(firstPoint, 2)
			veryLastPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
		end
	end

	-- Early exit: 2 points
	if numPoints == 2 then
		local spline = Spline.fromLine(
			ToTransform(zerothPoint, pointType),
			ToTransform(firstPoint, pointType),
			ToTransform(lastPoint, pointType),
			ToTransform(veryLastPoint, pointType)
		)

		return setmetatable({
			alpha = alpha,
			tension = tension,

			splines = {spline},
			domains = {0},

			length = spline.length
		}, CatRom)
	end

	-- Create splines
	local numSplines = numPoints - 1
	local splines = table.create(numSplines)
	local totalLength = 0

	-- Sliding window of control points to quicken instantiating splines
	local window1 = ToTransform(zerothPoint, pointType) -- FIX: Don't pack them in tables instead?
	local window2 = ToTransform(firstPoint, pointType)
	local window3 = ToTransform(points[2], pointType)
	local window4 = ToTransform(points[3] or veryLastPoint, pointType)

	-- Add the first spline, whose first point is not in the points table.
	splines[1] = Spline.new(window1, window2, window3, window4, alpha, tension)
	totalLength += splines[1].length

	-- Add the middle splines
	for i = 1, numPoints - 3 do
		-- Shift the window
		window1 = window2
		window2 = window3
		window3 = window4
		window4 = ToTransform(points[i + 3], pointType)

		local spline = Spline.new(window1, window2, window3, window4, alpha, tension)
		totalLength += spline.length
		splines[i + 1] = spline
	end

	-- Add the last spline, whose fourth point is not in the points table
	splines[numSplines] = Spline.new(window2, window3, window4,
		ToTransform(veryLastPoint, pointType), alpha, tension)
	totalLength += splines[numSplines].length

	-- Get the start of the domain interval for each spline
	local domains = table.create(numSplines - 1)
	local runningLength = 0
	for i, spline in splines do
		domains[i] = runningLength / totalLength
		runningLength += spline.length
	end

	return setmetatable({
		alpha = alpha,
		tension = tension,

		splines = splines,
		domains = domains,

		length = totalLength
	}, CatRom)
end

--[[
	Suppose we have a spline with n interpolants chained together. Denote the
	length of the i^th interpolant as L_i and the sum of the lengths as L.
	Consider a function f_i of the i^th interpolant (ex. SolvePosition) that
	we want to extend to a function f of the entire spline.

	To define f, we partition its domain [0, 1] into n smaller domains, where
	the i^th domain D_i has "length" L_i/L. Here is an example with n = 5:

	|       D1       |  D2  |     D3     |            D4            |    D5    |
	0 ------------------------------------------------------------------------ 1

	Then, to compute f(t), we find the domain D_j = [a, b] containing t and
	evaluate f_j at time (t - a) / (b - a).

	Using the notation above, the function below takes as input t and returns
	three outputs: the j^th interpolant, the time (t - a) / (b - a), and the
	index j. If t < 0, we let j = 1, and if t > 1, we let j = n.
]]
function CatRom:GetSplineAtTime(t: number)
	local splines = self.splines
	local domains = self.domains
	local numSplines = #splines

	-- There is only one option if there is one spline
	if numSplines == 1 then
		return splines[1], t
	end

	-- Special cases for when t is on the border or outside of [0, 1]
	if t < 0 then
		return splines[1], t / domains[1], 1
	elseif t == 0 then
		return splines[1], 0, 1
	elseif t == 1 then
		return splines[numSplines], 1, numSplines
	elseif t > 1 then
		return splines[numSplines], (t - domains[numSplines]) / (1 - domains[numSplines]), numSplines
	end

	-- Binary search for the spline containing t
	local left = 1
	local right = numSplines + 1

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = domains[mid]

		if t >= intervalStart then
			local intervalEnd = mid == numSplines and 1 or domains[mid + 1]

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
	error("Failed to get spline from t")
end

function CatRom:PrecomputeArcLengthParams(numIntervals: number?)
	numIntervals = numIntervals and math.max(1, math.round(numIntervals)) or DEFAULT_PRECOMPUTE_INTERVALS
	for _, spline in self.splines do
		spline:_PrecomputeArcLengthParams(numIntervals)
	end
end

function CatRom:SolveLength(a: number?, b: number?)
	-- Algorithm outline:
	-- Binary search for the two splines that contain the a and b positions
	-- Find where a is in the first spline
	-- Find where b is in the second spline
	-- Get the length from a to the end of the first spline
	-- Get the length from the start of the second spline to b
	-- Sum the full lengths of the splines between the a spline and b spline

	a = a or 0
	b = b or 1

	if a == 0 and b == 1 then
		return self.length
	end

	local splineA, splineAT, splineAIndex = self:GetSplineFromTime(a)
	local splineB, splineBT, splineBIndex = self:GetSplineFromTime(b)

	if splineAIndex == splineBIndex then
		return splineA:SolveLength(splineAT, splineBT)
	end

	local lengthA = splineA:SolveLength(splineAT, 1)
	local lengthB = splineB:SolveLength(0, splineBT)

	local intermediateLengths = 0
	for i = splineAIndex + 1, splineBIndex - 1 do
		intermediateLengths += self.splines[i].length
	end

	return lengthA + intermediateLengths + lengthB
end

function CatRom:SolvePosition(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolvePosition(splineTime)
end

function CatRom:SolveVelocity(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveVelocity(splineTime)
end

function CatRom:SolveAcceleration(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveAcceleration(splineTime)
end

function CatRom:SolveTangent(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveTangent(splineTime)
end

function CatRom:SolveNormal(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveNormal(splineTime)
end

function CatRom:SolveBinormal(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveBinormal(splineTime)
end

function CatRom:SolveCurvature(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCurvature(splineTime)
end

function CatRom:SolveCFrame(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCFrame(splineTime)
end

function CatRom:SolveRotCFrame(t: number)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveRotCFrame(splineTime)
end

function CatRom:SolveUnitSpeedPosition(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedPosition(splineTime)
end

function CatRom:SolveUnitSpeedVelocity(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedVelocity(splineTime)
end

function CatRom:SolveUnitSpeedAcceleration(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedAcceleration(splineTime)
end

function CatRom:SolveUnitSpeedTangent(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedTangent(splineTime)
end

function CatRom:SolveUnitSpeedNormal(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedNormal(splineTime)
end

function CatRom:SolveUnitSpeedBinormal(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedBinormal(splineTime)
end

function CatRom:SolveUnitSpeedCurvature(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedCurvature(splineTime)
end

function CatRom:SolveUnitSpeedCFrame(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedCFrame(splineTime)
end

function CatRom:SolveUnitSpeedRotCFrame(s: number)
	local spline, splineTime = self:GetSplineAtTime(s)
	return spline:SolveUnitSpeedRotCFrame(splineTime)
end

return CatRom