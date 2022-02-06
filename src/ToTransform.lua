--[[
	Inputs a Vector2, Vector3, or CFrame and returns the position of the input
	and the quaternion of the input's rotation (only applies to CFrame).
]]

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

	return nil
end

return ToTransform