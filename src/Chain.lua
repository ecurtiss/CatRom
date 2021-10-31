local FuzzyEq = require(script.Parent.FuzzyEq)
local Spline = require(script.Parent.Spline)
local Types = require(script.Parent.Types)

local Chain = {}
Chain.__index = Chain

-- Type checking
local tUnitInterval = Types.tUnitInterval
local tOptionalUnitInterval = Types.tOptionalUnitInterval

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

	local splines = table.create(numPoints - 1)
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
	local runningChainLength = 0
	for i, spline in ipairs(splines) do
		splineIntervals[i] = runningChainLength / chainLength
		runningChainLength += spline.Length
	end

	return setmetatable({
		ClassName = splines[1].ClassName .. "Chain",
		Length = chainLength,
		Points = points, -- FIX: Should this still include the first and last control points?
		Splines = splines,
		SplineIntervals = splineIntervals
	}, Chain)
end

-- Internal methods

-- Binary search for the spline in the chain where the alpha corresponds to.
-- Ex. For an alpha of 50%, we must find the spline in the chain that contains
-- the 50% mark.
local function AlphaToSpline(self, alpha: number)
	local splines = self.Splines
	local intervals = self.SplineIntervals
	local numSplines = #splines

	-- There is only one option if there is one spline.
	if numSplines == 1 then
		return splines[1], alpha
	end

	-- Special cases for when alpha is outside of the unti interval.
	if alpha < 0 then
		-- (alpha - 0) / (intervals[2] - 0)
		return splines[1], alpha / intervals[2]
	elseif alpha == 0 then
		return splines[1], 0
	elseif alpha == 1 then
		return splines[#splines], 1
	elseif alpha > 1 then
		local n = #splines
		return splines[n], (alpha - intervals[n]) / (1 - intervals[n])
	end

	-- Binary search for the spline containing the particular alpha along the chain.
	local left = 1
	local right = #splines

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = intervals[mid]

		if alpha >= intervalStart then
			local intervalEnd = mid == numSplines and 1 or intervals[mid + 1]

			if alpha <= intervalEnd then
				local spline = splines[mid]
				local splineAlpha = (alpha - intervalStart) / (intervalEnd - intervalStart)
				return spline, splineAlpha
			else
				left = mid + 1
			end
		else
			right = mid - 1
		end
	end

	-- This should theoretically never be possible, but if it occurs, then we
	-- should not fail silently.
	error("Failed to get spline from alpha")
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
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolvePosition(splineAlpha)
end
function Chain:SolveVelocity(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveVelocity(splineAlpha)
end
function Chain:SolveAcceleration(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveAcceleration(splineAlpha)
end
function Chain:SolveTangent(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveTangent(splineAlpha)
end
function Chain:SolveNormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveNormal(splineAlpha)
end
function Chain:SolveBinormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveBinormal(splineAlpha)
end
function Chain:SolveCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveCurvature(splineAlpha)
end
function Chain:SolveCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveCFrame(splineAlpha)
end
function Chain:SolveRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveRotCFrame(splineAlpha)
end
function Chain:SolveUniformPosition(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformPosition(splineAlpha)
end
function Chain:SolveUniformVelocity(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformVelocity(splineAlpha)
end
function Chain:SolveUniformAcceleration(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformAcceleration(splineAlpha)
end
function Chain:SolveUniformTangent(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformTangent(splineAlpha)
end
function Chain:SolveUniformNormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformNormal(splineAlpha)
end
function Chain:SolveUniformBinormal(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformBinormal(splineAlpha)
end
function Chain:SolveUniformCurvature(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformCurvature(splineAlpha)
end
function Chain:SolveUniformCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformCFrame(splineAlpha)
end
function Chain:SolveUniformRotCFrame(alpha: number)
	assert(tUnitInterval(alpha))
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveUniformRotCFrame(splineAlpha)
end
---- END GENERATED METHODS

return Chain