export type Knot = Vector2 | Vector3 | CFrame

-- I yanked this out of t because I hate submodules.
local t = {}

local function primitive(typeName)
	return function(value)
		local valueType = typeof(value)
		if valueType == typeName then
			return true
		else
			return false, string.format("%s expected, got %s", typeName, valueType)
		end
	end
end

t.callback = primitive("function")
t.Vector2 = primitive("Vector2")
t.Vector3 = primitive("Vector3")
t.CFrame = primitive("CFrame")

function t.optional(check)
	assert(t.callback(check))
	return function(value)
		if value == nil then
			return true
		end

		local success, errMsg = check(value)
		if success then
			return true
		else
			return false, string.format("(optional) %s", errMsg or "")
		end
	end
end

function t.number(value)
	local valueType = typeof(value)
	if valueType == "number" then
		if value == value then
			return true
		else
			return false, "unexpected NaN value"
		end
	else
		return false, string.format("number expected, got %s", valueType)
	end
end

function t.numberMax(max)
	return function(value)
		local success, errMsg = t.number(value)
		if not success then
			return false, errMsg
		end

		if value <= max then
			return true
		else
			return false, string.format("number <= %s expected, got %s", max, value)
		end
	end
end

function t.numberMin(min)
	return function(value)
		local success, errMsg = t.number(value)
		if not success then
			return false, errMsg or ""
		end

		if value >= min then
			return true
		else
			return false, string.format("number >= %s expected, got %s", min, value)
		end
	end
end

function t.numberConstrained(min, max)
	assert(t.number(min))
	assert(t.number(max))
	local minCheck = t.numberMin(min)
	local maxCheck = t.numberMax(max)

	return function(value)
		local minSuccess, minErrMsg = minCheck(value)
		if not minSuccess then
			return false, minErrMsg or ""
		end

		local maxSuccess, maxErrMsg = maxCheck(value)
		if not maxSuccess then
			return false, maxErrMsg or ""
		end

		return true
	end
end

local tUnitInterval = t.numberConstrained(0, 1)
local tOptionalUnitInterval = t.optional(tUnitInterval)
local function tKnots(k0: Knot, k1: Knot, k2: Knot, k3: Knot)
	local k0Type = t[typeof(k0)]
	return k0Type(k1) and k0Type(k2) and k0Type(k3)
end

return {
	tUnitInterval = tUnitInterval,
	tOptionalUnitInterval = tOptionalUnitInterval,
	tKnots = tKnots
}