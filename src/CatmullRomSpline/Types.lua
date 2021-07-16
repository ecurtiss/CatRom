local ReplicatedStorage = game:GetService("ReplicatedStorage")
local t = require(ReplicatedStorage.t)

export type Knot = Vector2 | Vector3 | CFrame

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