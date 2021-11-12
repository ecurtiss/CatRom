local FuzzyEq = require(script.FuzzyEq)
local Spline = require(script.Spline)
local ToTransform = require(script.ToTransform)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0

local CatRom = {}
CatRom.__index = CatRom

function CatRom.new(points, alpha, tension)
	alpha = alpha or DEFAULT_ALPHA -- Parameterization exponent
	tension = tension or DEFAULT_TENSION

	-- Typecheck
	assert(type(points) == "table", "Points must be a table.")
	assert(type(alpha) == "number", "Alpha must be a number.")
	assert(type(tension) == "number", "Tension must be a number.")
	local pointType = typeof(points[1])
	assert(#points >= 2, "Two or more points are required.")
	assert(pointType == "Vector2" or pointType == "Vector3" or pointType == "CFrame",
		"Points must be a table of Vector2s, Vector3s, or CFrames.")
	for _, point in ipairs(points) do
		assert(typeof(point) == pointType, "All points must be of the same type.")
	end

	-- Extrapolate to get 0th and n+1th points
	local numPoints = #points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	local zerothPoint, veryLastPoint do
		if FuzzyEq(firstPoint, lastPoint) then
			-- Loops
			zerothPoint = points[numPoints - 1]
			veryLastPoint = points[2]
		else
			-- Does not loop
			zerothPoint = points[2]:Lerp(firstPoint, 2)
			veryLastPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
		end
	end

	-- Stream of control points to quicken instantiating splines. I benchmarked
	-- this to be about 4x faster than copying all of the points into a table
	-- and doing 4 table lookups per instantiation.
	local stream1 = ToTransform(zerothPoint, pointType) -- FIX: Don't pack them in tables instead?
	local stream2 = ToTransform(firstPoint, pointType)
	local stream3 = ToTransform(points[2], pointType)
	local stream4 = ToTransform(points[3], pointType)

	local numSplines = numPoints - 1
	local splines = table.create(numSplines)
	local totalLength = 0

	-- Add the first spline, whose first point is not in the points table
	splines[1] = Spline.new(stream1, stream2, stream3, stream4, alpha, tension)
	totalLength += splines[1].length

	-- Add the middle splines
	for i = 1, numPoints - 3 do
		-- Shift the stream
		stream1 = stream2
		stream2 = stream3
		stream3 = stream4
		stream4 = ToTransform(points[i + 3], pointType)

		local spline = Spline.new(stream1, stream2, stream3, stream4, alpha, tension)
		totalLength += spline.length
		splines[i + 1] = spline
	end

	-- Add the last spline, whose fourth point is not in the points table
	splines[numSplines] = Spline.new(stream2, stream3, stream4,
		ToTransform(veryLastPoint, pointType), alpha, tension)
	totalLength += splines[numSplines].length

	-- Get the start of the domain interval for each spline
	local domains = table.create(numSplines - 1)
	local runningLength = 0
	for i, spline in ipairs(splines) do
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

-- Binary search for the spline in the chain that the alpha corresponds to.
-- Ex. For an alpha of 50%, we must find the spline in the chain that contains
-- the 50% mark.
local function AlphaToSpline(self, alpha)
	local splines = self.splines
	local domains = self.domains
	local numSplines = #splines

	-- There is only one option if there is one spline
	if numSplines == 1 then
		return splines[1], alpha
	end

	-- Special cases for when alpha is on the border or outside of the unit
	-- interval
	if alpha < 0 then
		return splines[1], alpha / domains[1]
	elseif alpha == 0 then
		return splines[1], 0
	elseif alpha == 1 then
		return splines[numSplines], 1
	elseif alpha > 1 then
		return splines[numSplines], (alpha - domains[numSplines]) / (1 - domains[numSplines])
	end

	-- Binary search for the spline containing the particular alpha along the
	-- chain
	local left = 1
	local right = numSplines + 1

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = domains[mid]

		if alpha >= intervalStart then
			local intervalEnd = mid == numSplines and 1 or domains[mid + 1]

			if alpha <= intervalEnd then
				local spline = splines[mid]
				-- splineAlpha is an estimate along the spline of where alpha
				-- falls.
				-- FIX: Improve the accuracy of splineAlpha
				local splineAlpha = (alpha - intervalStart) / (intervalEnd - intervalStart)
				return spline, splineAlpha, mid
			else
				left = mid + 1
			end
		else
			right = mid - 1
		end
	end

	-- This should theoretically never be possible, but if it occurs, then we
	-- should not fail silently
	error("Failed to get spline from alpha")
end

function CatRom:SolveLength(a, b)
	if not a and not b or a == 0 and b == 1 then
		return self.length
	end

	a = a or 0
	b = b or 1

	-- splineAAlpha and splineBAlpha are estimates. This should be made
	-- more accurate in the future
	local splineA, splineAAlpha, splineAIndex = AlphaToSpline(a)
	local splineB, splineBAlpha, splineBIndex = AlphaToSpline(b)

	local lengthA = splineA:SolveLength(splineAAlpha, 1)
	local lengthB = splineB:SolveLength(0, splineBAlpha)

	local intermediateLengths = 0
	for i = splineAIndex + 1, splineBIndex - 1 do
		intermediateLengths += self.splines[i].length
	end

	return lengthA + intermediateLengths + lengthB

	-- Algorithm outline:
	-- Binary search for the two splines that contain the a and b positions.
	-- Find where a is in the first spline
	-- Find where b is in the second spline
	-- Get the length from a to the end of the first spline
	-- Get the length from the start of the second spline to b
	-- Sum the full lengths of the splines between the a spline and b spline
end

-- TODO: SolveArcLength

---- START GENERATED METHODS
function CatRom:SolvePosition(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolvePosition(splineAlpha)
end
function CatRom:SolveVelocity(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveVelocity(splineAlpha)
end
function CatRom:SolveAcceleration(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveAcceleration(splineAlpha)
end
function CatRom:SolveTangent(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveTangent(splineAlpha)
end
function CatRom:SolveNormal(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveNormal(splineAlpha)
end
function CatRom:SolveBinormal(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveBinormal(splineAlpha)
end
function CatRom:SolveCurvature(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveCurvature(splineAlpha)
end
function CatRom:SolveCFrame(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveCFrame(splineAlpha)
end
function CatRom:SolveRotCFrame(alpha: number)
	local spline, splineAlpha = AlphaToSpline(self, alpha)
	return spline:SolveRotCFrame(splineAlpha)
end
---- END GENERATED METHODS

return CatRom