--[[
	Inputs a Vector2, Vector3, or CFrame and returns the position of the input
	and the quaternion of the input's rotation (only applies to CFrame).
]]

local function CFrameToQuaternion(cframe)
	local _, _, _, m00, m01, m02, m10, m11, m12, m20, m21, m22 = cframe:GetComponents()
	local trace = m00 + m11 + m22
	if trace > 0 then
		local s = math.sqrt(1 + trace)
		local reciprocal = 0.5 / s
		return {
			s / 2,
			(m21 - m12) * reciprocal,
			(m02 - m20) * reciprocal,
			(m10 - m01) * reciprocal
		}
	else
		local big = math.max(m00, m11, m22)
		if big == m00 then
			local s = math.sqrt(1 + m00 - m11 - m22)
			local reciprocal = 0.5 / s
			return {
				(m21 - m12) * reciprocal,
				s / 2,
				(m10 + m01) * reciprocal,
				(m02 + m20) * reciprocal
			}
		elseif big == m11 then
			local s = math.sqrt(1 - m00 + m11 - m22)
			local reciprocal = 0.5 / s
			return {
				(m02 - m20) * reciprocal,
				(m10 + m01) * reciprocal,
				s / 2,
				(m21 + m12) * reciprocal
			}
		elseif big == m22 then
			local s = math.sqrt(1 - m00 - m11 + m22 )
			local reciprocal = 0.5 / s
			return {
				(m10 - m01) * reciprocal,
				(m02 + m20) * reciprocal,
				(m21 + m12) * reciprocal,
				s / 2
			}
		else
			return table.create(4, nil)
		end
	end
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