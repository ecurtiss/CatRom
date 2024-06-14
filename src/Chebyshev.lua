local Types = require(script.Parent.Types)

local SOLVE_TOLERANCE = 1e-4
local GET_SOLVE_BOUNDS_TOLERANCE = 1e-6
local MAX_REGULA_FALSI_ITERATIONS = 10

--[=[
	@class Chebyshev
	@ignore
	
	Interpolates the arc length function of a spline segment using a Chebyshev
	polynomial. Extracted from work by DarkInfernoDrago
	(https://github.com/rbxmath/rbxmath).
]=]
local Chebyshev: Types.ChebyshevMt = {} :: Types.ChebyshevMt
Chebyshev.__index = Chebyshev

local transformedChebyshevGridCache = {}

-- Maps [-1, 1] -> [0, 1]
local function transformToSplineDomain(x: number): number
	return x / 2 + 0.5
end

-- Gets a Chebyshev grid with degree + 1 points, transformed from [-1, 1] to
-- [0, 1]
local function getTransformedChebyshevGrid(degree: number): {number}
	if transformedChebyshevGridCache[degree] then
		return transformedChebyshevGridCache[degree]
	end
	
	local numGridPoints = degree + 1
	local result = table.create(numGridPoints)
	for i = 0, degree do
		result[numGridPoints - i] = transformToSplineDomain(math.cos(i * math.pi / degree))
	end

	transformedChebyshevGridCache[degree] = result

	return result
end

function Chebyshev.new(f: (number) -> number, degree: number)
	local numGridPoints = degree + 1
	local grid = getTransformedChebyshevGrid(degree)
	
	local gridValues = table.create(numGridPoints)
	gridValues[1] = 0
	gridValues[numGridPoints] = 1
	for i = 2, numGridPoints do
		gridValues[i] = f(grid[i])
	end

	return setmetatable({
		grid = grid,
		gridValues = gridValues,
	}, Chebyshev)
end

--- Computes p(x) using the second barycentric interpolation formula
function Chebyshev:Evaluate(x: number): number
	local grid = self.grid
	local numGridPoints = #grid
	local gridValues = self.gridValues

	local xDiffList = table.create(numGridPoints)
	for i, gridPoint in grid do
		local diff = x - gridPoint
		if diff == 0 then
			return gridValues[i]
		end
		xDiffList[i] = (-1) ^ (i - 1) / diff
	end

	local val
	local numerator = 0
	local denominator = 1 / (2 * x)

	for i = 2, numGridPoints - 1 do
		val = xDiffList[i]
		numerator += gridValues[i] * val
		denominator += val
	end

	val = xDiffList[numGridPoints] / 2
	numerator += val
	denominator += val

	return numerator / denominator
end

--- Returns the two consecutive grid points whose values bound y, as well as
--- their values
function Chebyshev:GetSolveBounds(y: number): (number, number, number, number)
	local numGridPoints = #self.grid
	local gridValues = self.gridValues
	local leftGridValue = 0

	if math.abs(y) < SOLVE_TOLERANCE then
		return 0, 0, -y, -y
	end
	
	for i = 2, numGridPoints do
		local rightGridValue = gridValues[i]

		if math.abs(y - rightGridValue) < GET_SOLVE_BOUNDS_TOLERANCE then
			return self.grid[i], self.grid[i], rightGridValue - y, rightGridValue - y
		elseif y > leftGridValue and y < rightGridValue then
			return self.grid[i - 1], self.grid[i], leftGridValue - y, rightGridValue - y
		end

		leftGridValue = rightGridValue
	end

	error("Failed to get bounds for Solve")
end

--- Solves the problem p(x) = y for x given y via regula falsi. Assumes that the
--- function is strictly increasing.
function Chebyshev:Solve(y: number): number
	if y == 0 or y == 1 then
		return y
	end

	local leftBound, rightBound, leftValue, rightValue = self:GetSolveBounds(y)

	if leftBound == rightBound then
		-- y is a value on the Chebyshev grid
		return leftBound
	end

	for _ = 1, MAX_REGULA_FALSI_ITERATIONS do
		local t = (leftBound * rightValue - rightBound * leftValue) / (rightValue - leftValue)
		
		-- Guard against floating point errors
		if t < leftBound or t > rightBound then
			t = (leftBound + rightBound) / 2
		end
		
		local f = self:Evaluate(t) - y
		
		if math.abs(f) < SOLVE_TOLERANCE then
			return t
		elseif f > 0 then
			rightBound = t
			rightValue = f
		else
			leftBound = t
			leftValue = f
		end
	end

	warn("Reached maximum regula falsi iterations")
	return (leftBound + rightBound) / 2
end

--- Inverts the Chebyshev polynomial by sampling the grid points on its inverse.
function Chebyshev:Invert()
	local grid = self.grid
	local numGridPoints = #grid
	
	local gridValues = table.create(numGridPoints)
	gridValues[1] = 0
	gridValues[numGridPoints] = 1
	for i = 2, numGridPoints do
		gridValues[i] = self:Solve(grid[i])
	end

	return setmetatable({
		grid = self.grid,
		gridValues = gridValues,
	}, Chebyshev)
end

return Chebyshev