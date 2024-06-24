local Types = require(script.Parent.Types)

--[=[
	@class Utils
	@ignore

	Utility functions
]=]
local Utils = {}

--[=[
	Solves for the real roots of the quadratic polynomial at^2 + bt + c. Returns
	nil if there are no real roots. Returns only one real root if the two real
	roots are equal.
]=]
function Utils.SolveQuadratic(a: number, b: number, c: number): (number?, number?)
	if a == 0 then
		if b == 0 then
			return nil
		else
			return -c / b
		end
	end

	local discriminant = b ^ 2 - 4 * a * c

	if discriminant == 0 then
		return -b / (2 * a), nil
	elseif discriminant > 0 then
		local negativeB = -b
		local sqrt = math.sqrt(discriminant)
		local doubleA = 2 * a

		return (negativeB - sqrt) / doubleA, (negativeB + sqrt) / doubleA
	end

	return nil, nil
end

-- Converts cframe.Rotation into a quaternion in {w, x, y, z} format, where
-- w is the angle and (x, y, z) is the axis.
function Utils.CFrameToQuaternion(cframe: CFrame): Types.Quaternion
	local axis, angle = cframe:ToAxisAngle()
	angle /= 2
	axis = math.sin(angle) * axis
	return {math.cos(angle), axis.X, axis.Y, axis.Z}
end

--- Extracts the position and rotation of a point. Only points of type CFrame
--- have a rotational component.
function Utils.SeparatePositionAndRotation(point: Types.Point, pointType: Types.PointType): (Types.Point, Types.Quaternion?)
	if pointType == "CFrame" then
		return (point :: CFrame).Position, Utils.CFrameToQuaternion(point :: CFrame)
	else
		return point
	end
end

function Utils.Lerp(a: Types.Point, b: Types.Point, t: number): Types.Point
	return if typeof(a) == "number" then a + (b - a) * t else a:Lerp(b, t)
end

function Utils.FuzzyEq(a: Types.Point, b: Types.Point, eps: number): boolean
	return if typeof(a) == "number"
		then a == b or math.abs(a - b) <= (math.abs(a) + 1) * eps
		else a:FuzzyEq(b, eps)
end

function Utils.Magnitude(v: Types.Vector): number
	return if typeof(v) == "number" then math.abs(v) else v.Magnitude
end

function Utils.Unit(v: Types.Vector): Types.Vector
	return if typeof(v) == "number" then math.sign(v) else v.Unit
end

function Utils.Dot(a: Types.Vector, b: Types.Vector): number
	return if typeof(a) == "number" then a * b else a:Dot(b)
end

function Utils.Min(points: {Types.Vector}): Types.Vector
	local firstPoint = points[1]
	return if typeof(firstPoint) == "number"
		then math.min(table.unpack(points))
		else firstPoint:Min(table.unpack(points))
end

function Utils.Max(points: {Types.Vector}): Types.Vector
	local firstPoint = points[1]
	return if typeof(firstPoint) == "number"
		then math.max(table.unpack(points))
		else firstPoint:Max(table.unpack(points))
end

return Utils