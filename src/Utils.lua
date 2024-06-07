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
	if pointType == "Vector2" or pointType == "Vector3" then
		return point
	elseif pointType == "CFrame" then
		return point.Position, Utils.CFrameToQuaternion(point)
	else
		error("Bad inputs")
	end
end

return Utils