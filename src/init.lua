local Spline = require(script.Spline)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_PRECOMPUTE_INTERVALS = 16

type Point = CFrame | Vector2 | Vector3

local CatRom = {}
CatRom.__index = CatRom

-- Converts cframe.Rotation into a quaternion in {w, x, y, z} format, where
-- w is the angle and (x, y, z) is the axis.
local function CFrameToQuaternion(cframe)
	local axis, angle = cframe:ToAxisAngle()
	angle /= 2
	axis = math.sin(angle) * axis
	return {math.cos(angle), axis.X, axis.Y, axis.Z}
end

-- Extracts the position and rotation of a point. Only points of type CFrame
-- have a rotational component.
local function ToTransform(point, pointType)
	if pointType == "Vector2" or pointType == "Vector3" then
		return {point}
	elseif pointType == "CFrame" then
		return {point.Position, CFrameToQuaternion(point)}
	end

	error("Bad inputs")
end

function CatRom.new(points: {Point}, alpha: number?, tension: number?)
	alpha = alpha or DEFAULT_ALPHA -- Parametrization exponent
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
			if not point:FuzzyEq(prevPoint) then
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

	-- Special cases for when t is on the boundary or outside of [0, 1]
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
	error("Failed to get spline")
end

function CatRom:PrecomputeArcLengthParams(numIntervals: number?)
	numIntervals = if numIntervals then math.max(1, math.round(numIntervals)) else DEFAULT_PRECOMPUTE_INTERVALS
	for _, spline in self.splines do
		spline:_PrecomputeArcLengthParams(numIntervals)
	end
end

function CatRom:SolveLength(a: number?, b: number?)
	a = a or 0
	b = b or 1

	if a == 0 and b == 1 then
		return self.length
	end

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

function CatRom:SolvePosition(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolvePosition(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveVelocity(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveVelocity(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveAcceleration(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveAcceleration(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveTangent(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveTangent(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveNormal(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveNormal(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveBinormal(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveBinormal(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveCurvature(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCurvature(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveCFrame(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveCFrame(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

function CatRom:SolveRotCFrame(t: number, unitSpeed: boolean?)
	local spline, splineTime = self:GetSplineAtTime(t)
	return spline:SolveRotCFrame(if unitSpeed then spline:Reparametrize(splineTime) else splineTime)
end

-- Utility function for calling a method on n uniformly-spaced points in [a, b].
-- TODO: Bulk GetSplineAtTime
-- TODO: Bulk ReparametrizeHybrid
function CatRom:SolveBulk(f: (table, number) -> any, n: number, a: number?, b: number?)
	n = math.round(n)
	a = a or 0
	b = b or 1

	if a == b then
		return table.create(f(self, a), n)
	elseif n < 1 then
		return
	elseif n == 1 then
		return {f(self, a)}
	end

	local outputs = table.create(n)
	outputs[1] = f(self, a)
	outputs[n] = f(self, b)

	local increment = (b - a) / (n - 1)
	for i = 1, n - 2 do
		outputs[i + 1] = f(self, a + increment * i)
	end

	return outputs
end

return CatRom