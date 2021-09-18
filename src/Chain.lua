local Spline = require(script.Parent.Spline)
local Types = require(script.Parent.Types)

local Chain = {}
Chain.__index = Chain

local EPSILON = 1e-4

-- Type checking
local tUnitInterval = Types.tUnitInterval
local tOptionalUnitInterval = Types.tOptionalUnitInterval

local function FuzzyEq(a, b)
	local aType = typeof(a)

	if aType == "number" then
		return a == b or math.abs(a - b) <= (math.abs(a) + 1) * EPSILON
	elseif aType == "Vector3" then
		return a:FuzzyEq(b, EPSILON)
	elseif aType == "Vector2" then
		local aX, bX = a.X, b.X
		if aX == bX or math.abs(aX - bX) <= (math.abs(aX) + 1) * EPSILON then
			local aY, bY = a.Y, b.Y
			if aY == bY or math.abs(aY - bY) <= (math.abs(aY) + 1) * EPSILON then
				return true
			end
		end
	elseif aType == "CFrame" then
		if a.Position:FuzzyEq(b.Position, EPSILON)
			and a.RightVector:FuzzyEq(b.RightVector, EPSILON)
			and a.UpVector:FuzzyEq(b.UpVector, EPSILON)
			and a.LookVector:FuzzyEq(b.LookVector, EPSILON) then
			return true
		end
	end

	return false
end

-- Constructor
function Chain.new(points: {Types.Knot}, alpha: number?, tension: number?)
	assert(#points >= 2, "2 or more points are required to create a chain.")

	local numPoints = #points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	if FuzzyEq(firstPoint, lastPoint) then -- loops
		table.insert(points, points[2]) -- last control point
		table.insert(points, 1, points[numPoints - 1]) -- first control point
	else
		table.insert(points, points[numPoints - 1]:Lerp(lastPoint, 2)) -- last control point
		table.insert(points, 1, points[2]:Lerp(firstPoint, 2)) -- first control point
	end

	local splines = table.create(numPoints + 1)
	local chainLength = 0
	for i = 1, numPoints - 1 do
		local spline = Spline.new(
			points[i],
			points[i + 1],
			points[i + 2],
			points[i + 3],
			alpha,
			tension
		)
		splines[i] = spline
		chainLength += spline.Length
	end

	local splineIntervals = table.create(numPoints - 1)
	local splineFromAlphaCache = table.create(100)
	local runningChainLength = 0
	for i, spline in ipairs(splines) do
		local intervalStart = runningChainLength / chainLength
		runningChainLength += spline.Length
		local intervalEnd = runningChainLength / chainLength
		splineIntervals[i] = {Start = intervalStart, End = intervalEnd}
		local endAlpha = math.floor(intervalEnd * 100) - math.ceil(intervalStart * 100)
		for alpha = 0, endAlpha do
			local newAlpha = math.ceil(intervalStart * 100) / 100 + alpha / 100
			splineFromAlphaCache[string.format("%.2f", newAlpha)] = i
		end
	end

	return setmetatable({
		ClassName = splines[1].ClassName .. "Chain",
		Length = chainLength,
		Points = points, -- FIX: Should this still include the first and last control points?
		Splines = splines,
		SplineFromAlphaCache = splineFromAlphaCache,
		SplineIntervals = splineIntervals
	}, Chain)
end

-- Internal methods
function Chain:_AlphaToSpline(alpha: number)
	local startInterval = self.SplineFromAlphaCache[string.format("%.2f", alpha)]
	local splineIntervals = self.SplineIntervals

	for i = startInterval, #splineIntervals do
		local splineInterval = splineIntervals[i]
		if alpha >= splineInterval.Start and alpha <= splineInterval.End then
			local splineAlpha = (alpha - splineInterval.Start) / (splineInterval.End - splineInterval.Start)
			return self.Splines[i], splineAlpha, splineInterval
		end
	end
end

-- Methods
function Chain:SolveLength(a: number?, b: number?)
	if not a and not b then return self.Length end
	assert(tOptionalUnitInterval(a))
	assert(tOptionalUnitInterval(b))
	a = a or 0
	b = b or 1
	local length = 0
	-- TODO: implement this
end

function Chain:SolveArcLength(a: number?, b: number?)
	if not a and not b then return self.Length end
	assert(tOptionalUnitInterval(a))
	assert(tOptionalUnitInterval(b))
	a = a or 0
	b = b or 1
	local length = 0
	-- TODO: implement this
end

---- START GENERATED METHODS
function Chain:SolvePosition(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolvePosition(splineAlpha)
end
function Chain:SolveVelocity(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveVelocity(splineAlpha)
end
function Chain:SolveAcceleration(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveAcceleration(splineAlpha)
end
function Chain:SolveTangent(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveTangent(splineAlpha)
end
function Chain:SolveNormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveNormal(splineAlpha)
end
function Chain:SolveBinormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveBinormal(splineAlpha)
end
function Chain:SolveCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveCurvature(splineAlpha)
end
function Chain:SolveCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveCFrame(splineAlpha)
end
function Chain:SolveRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveRotCFrame(splineAlpha)
end
function Chain:SolveUniformPosition(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformPosition(splineAlpha)
end
function Chain:SolveUniformVelocity(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformVelocity(splineAlpha)
end
function Chain:SolveUniformAcceleration(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformAcceleration(splineAlpha)
end
function Chain:SolveUniformTangent(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformTangent(splineAlpha)
end
function Chain:SolveUniformNormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformNormal(splineAlpha)
end
function Chain:SolveUniformBinormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformBinormal(splineAlpha)
end
function Chain:SolveUniformCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformCurvature(splineAlpha)
end
function Chain:SolveUniformCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformCFrame(splineAlpha)
end
function Chain:SolveUniformRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = self:_AlphaToSpline(alpha)
	return spline:SolveUniformRotCFrame(splineAlpha)
end
---- END GENERATED METHODS

return Chain